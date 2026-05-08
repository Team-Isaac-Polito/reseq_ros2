import time as walltime
import traceback
from bisect import bisect_left
from collections import deque
from math import atan2, cos, pi, sin

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

"""ROS node with follow-the-leader control for a snake-like modular robot.

Each rear module follows the exact path the head traced, with a spatial
delay of one module_spacing per joint. The head's heading is tracked via
dead-reckoning from cmd_vel, and stored in a distance-indexed path buffer.

Joint angle computation (follow-the-leader):
    For joint j between module j and module j+1:
        front_heading = heading at (head_distance - j * module_spacing)
        rear_heading  = heading at (head_distance - (j+1) * module_spacing)
        joint_angle   = rear_heading - front_heading

    This directly gives the URDF joint value: negative for left turns
    (rear module hasn't turned as much yet → points right relative to front).

The AGEVAR kinematic model computes per-module velocities using the
joint angles to account for the geometric coupling between modules.
"""

# Joint limits from URDF (±π/4 = ±0.785 rad)
YAW_LIMIT = pi / 4


class Agevar(Node):
    def __init__(self):
        super().__init__('agevar')

        # Declaring parameters and getting values
        self.a = self.declare_parameter('a', 0.0).get_parameter_value().double_value
        self.b = self.declare_parameter('b', 0.0).get_parameter_value().double_value
        self.modules = (
            self.declare_parameter('modules', [0]).get_parameter_value().integer_array_value
        )

        # create the enable/disable service
        self.enabled = True
        self.create_service(SetBool, '/agevar/enable', self.handle_enable)

        self.n_mod = len(self.modules)
        self.n_joints = self.n_mod - 1
        self.module_spacing = self.a + self.b

        # Actual yaw angles from joint_states (for monitoring)
        self.yaw_angles = [0.0] * self.n_mod

        # Follow-the-leader path tracking
        # Head heading tracked via dead-reckoning from cmd_vel
        self.head_theta = 0.0  # accumulated heading (radians)
        self.head_distance = 0.0  # cumulative distance traveled

        # Path buffer: parallel lists for O(1) append + O(log n) binary search
        # _dist[i] is monotonically increasing, _heading[i] is the head heading at that distance
        self._dist = deque(maxlen=5000)
        self._heading = deque(maxlen=5000)
        self._dist.append(0.0)
        self._heading.append(0.0)

        # Backward path buffer — rebuilt fresh on each forward→backward switch.
        self._bwd_dist = deque(maxlen=5000)
        self._bwd_heading = deque(maxlen=5000)
        self._bwd_distance = 0.0
        self._bwd_head_theta = 0.0
        self._bwd_dist.append(0.0)
        self._bwd_heading.append(0.0)

        # Smoothed yaw commands per joint (stores URDF joint angles)
        self.yaw_commands = [0.0] * self.n_joints
        self.adaptive_alpha = [1.0] * self.n_joints
        self.smooth_alpha = 0.7

        # Track previous sign to detect direction changes and flush the path buffer
        self._prev_sign = 1

        # Time tracking (with wall-clock fallback for sim time startup)
        self.last_time = None
        self.last_wall = None
        self.clock_valid = False  # set True once sim clock returns > 0

        # subscribe to remote (parsed by teleop_twist_joy)
        self.create_subscription(Twist, '/cmd_vel', self.remote_callback, 10)

        self.latest_feedback = None
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)

        self.controller_pubs = []
        for i in range(self.n_mod):
            # create publisher for the diff drive controllers
            pub = self.create_publisher(TwistStamped, f'/diff_controller{i + 1}/cmd_vel', 10)
            self.controller_pubs.append(pub)

        # create publishers for yaw joint controllers (ForwardCommandController)
        self.yaw_pubs = []
        for i in range(self.n_joints):
            pub = self.create_publisher(Float64MultiArray, f'/yaw_controller{i + 2}/commands', 10)
            self.yaw_pubs.append(pub)

    def handle_enable(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self.enabled = request.data

        if not self.enabled:
            for pub in self.controller_pubs:
                pub.publish(TwistStamped())  # stop all drive controllers

        response.success = True
        response.message = 'Agevar node enabled' if self.enabled else 'Agevar node disabled'
        self.get_logger().info(response.message)
        return response

    def _compute_dt(self):
        """Compute time step, using wall clock as fallback when sim clock is stuck at 0."""
        now = self.get_clock().now()
        wall_now = walltime.monotonic()

        if self.last_time is None:
            self.last_time = now
            self.last_wall = wall_now
            return 0.0

        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Detect sim clock stuck at 0
        if dt <= 0 or (not self.clock_valid and now.nanoseconds == 0):
            # Use wall clock as fallback
            dt = wall_now - self.last_wall

        if now.nanoseconds > 0:
            self.clock_valid = True

        self.last_wall = wall_now

        # Safety cap: discard unreasonably large gaps (e.g. sim clock jumps)
        return min(dt, 0.5)

    def remote_callback(self, msg: Twist):
        if not self.enabled:
            self.get_logger().debug('Agevar node is disabled, ignoring command')
            return

        # extract information from ROS Twist message
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        sign = 1 if linear_vel >= 0 else -1

        # On direction reversal, reset the appropriate path buffer.
        if sign != self._prev_sign:
            if sign == -1:  # forward → backward: last module becomes physical head
                # Compute the last module's current heading from the forward path buffer.
                last_lag_dist = self.head_distance - self.n_joints * self.module_spacing
                self._bwd_head_theta = self._interp_heading(last_lag_dist)

                # Reconstruct the backward buffer from the current joint commands
                self._bwd_distance = 0.0
                self._bwd_dist.clear()
                self._bwd_heading.clear()
                entries_bwd = []
                cumulative_bwd = self._bwd_head_theta
                for j in range(self.n_joints + 1):
                    entries_bwd.append((-j * self.module_spacing, cumulative_bwd))
                    if j < self.n_joints:
                        cumulative_bwd += self.yaw_commands[self.n_joints - 1 - j]
                
                # Append in ascending distance order
                for d, h in reversed(entries_bwd):
                    self._bwd_dist.append(d)
                    self._bwd_heading.append(h)

            else:
                # Update head_theta to the first module's actual current heading.
                first_lag_dist = self._bwd_distance - self.n_joints * self.module_spacing
                if first_lag_dist >= 0.0:
                    self.head_theta = self._interp_heading(
                        first_lag_dist, self._bwd_dist, self._bwd_heading
                    )
                
                # Reconstruct the forward path buffer from the current joint commands.
                self._dist.clear()
                self._heading.clear()
                entries = []
                cumulative_h = self.head_theta
                for j in range(self.n_joints + 1):
                    entries.append((self.head_distance - j * self.module_spacing, cumulative_h))
                    if j < self.n_joints:
                        cumulative_h += self.yaw_commands[j]
                
                # Append in ascending distance order
                for d, h in reversed(entries):
                    self._dist.append(d)
                    self._heading.append(h)
            self._prev_sign = sign

        dt = self._compute_dt()
        if dt <= 0:
            return

        self.update_yaw_angles()

        abs_v = abs(linear_vel)
        abs_w = abs(angular_vel)

        is_rotating = abs_w > 0.05
        is_moving = abs_v > 0.01

        if is_rotating or is_moving:
            if sign == 1:
                # Forward: dead-reckon first module heading from cmd_vel.
                self.head_theta += angular_vel * dt
                if is_moving:
                    self.head_distance += abs_v * dt
                else:
                    # Pure rotation: virtual advance so the buffer fills.
                    virtual_v = abs_w * self.module_spacing * 2.0
                    self.head_distance += virtual_v * dt
                
                self._dist.append(self.head_distance)
                self._heading.append(self.head_theta)
                max_lookback = (self.n_joints + 1) * self.module_spacing + 0.5
                min_needed = self.head_distance - max_lookback
                while len(self._dist) > 2 and self._dist[1] < min_needed:
                    self._dist.popleft()
                    self._heading.popleft()
            else:
                # Backward: dead-reckon last module (new head) heading into its own buffer.
                self._bwd_head_theta += angular_vel * dt
                if is_moving:
                    self._bwd_distance += abs_v * dt
                else:
                    # Pure rotation: virtual advance so the buffer fills.
                    virtual_v = abs_w * self.module_spacing * 2.0
                    self._bwd_distance += virtual_v * dt
                
                self._bwd_dist.append(self._bwd_distance)
                self._bwd_heading.append(self._bwd_head_theta)
                max_lookback = (self.n_joints + 1) * self.module_spacing + 0.5
                min_needed = self._bwd_distance - max_lookback
                while len(self._bwd_dist) > 2 and self._bwd_dist[1] < min_needed:
                    self._bwd_dist.popleft()
                    self._bwd_heading.popleft()

            # Compute joint angles via follow-the-leader heading differences.
            for j in range(self.n_joints):
                if is_moving:
                    if sign == 1:
                        # Forward: followers lag behind (lower accumulated distance).
                        front_dist = self.head_distance - j * self.module_spacing
                        rear_dist = self.head_distance - (j + 1) * self.module_spacing
                        front_heading = self._interp_heading(front_dist)
                        rear_heading = self._interp_heading(rear_dist)
                    else:
                        # Backward: followers lag behind the last module in backward-distance.
                        front_dist = self._bwd_distance - j * self.module_spacing
                        rear_dist = self._bwd_distance - (j + 1) * self.module_spacing
                        front_heading = self._interp_heading(
                            front_dist, self._bwd_dist, self._bwd_heading
                        )
                        rear_heading = self._interp_heading(
                            rear_dist, self._bwd_dist, self._bwd_heading
                        )

                    # Joint angle = heading difference (negative for left turns)
                    raw_angle = rear_heading - front_heading

                    # Normalize to [-π, π]
                    raw_angle = atan2(sin(raw_angle), cos(raw_angle))

                    # Clamp to joint limits
                    raw_angle = max(-YAW_LIMIT, min(YAW_LIMIT, raw_angle))
                else:
                    # Pure rotation: saturate all joints to YAW_LIMIT in turn direction
                    turn_dir = 1.0 if angular_vel > 0 else -1.0
                    raw_angle = turn_dir * YAW_LIMIT

                # In backward mode, joint j from the last module's perspective maps to
                # physical joint in the yaw_commands array.
                cmd_idx = j if sign == 1 else (self.n_joints - 1 - j)

                # Compute tracking error using encoder feedback
                error = self.yaw_commands[cmd_idx] - self.yaw_angles[cmd_idx]
                error_mag = abs(error)

                # Update per‑joint adaptive alpha
                self.adaptive_alpha[cmd_idx] = self._compute_adaptive_alpha(error_mag)

                # Apply smoothing using per‑joint alpha
                alpha = self.adaptive_alpha[cmd_idx]
                delta = raw_angle - self.yaw_commands[cmd_idx]
                
                self.yaw_commands[cmd_idx] += alpha * delta

                yaw_msg = Float64MultiArray()
                yaw_msg.data = [self.yaw_commands[cmd_idx]]
                self.yaw_pubs[cmd_idx].publish(yaw_msg)

        # Compute per-module velocities using AGEVAR kinematic model
        modules = list(range(self.n_mod))
        if sign == -1:  # going backwards
            modules.reverse()
            linear_vel = -linear_vel
            angular_vel = -angular_vel

        for mod_id in modules:
            out_msg = TwistStamped()
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.twist.linear.x = sign * linear_vel
            out_msg.twist.angular.z = sign * angular_vel

            if self.enabled:
                self.controller_pubs[mod_id].publish(out_msg)

            if mod_id != modules[-1]:  # for every module except the last one
                if sign == 1:
                    joint_idx = mod_id
                else:
                    # Backward: modules iterate [n-1, n-2, ..., 0].
                    joint_idx = mod_id - 1

                if 0 <= joint_idx < self.n_joints:
                    # Kinematic model uses heading change (positive = left turn)
                    # yaw_commands stores URDF angle (negative = left turn)
                    yaw_angle = -sign * self.yaw_commands[joint_idx]
                else:
                    yaw_angle = 0.0

                # compute linear and angular velocity of the following module
                linear_vel, angular_vel = self.kinematic(linear_vel, angular_vel, yaw_angle)

                self.get_logger().debug(f'Output lin:{linear_vel}, ang:{angular_vel}, sign:{sign}')

    def _interp_heading(self, target_dist, dist_buf=None, heading_buf=None):
        """Interpolate heading from a path buffer using binary search.

        Uses the forward buffer (_dist/_heading) by default; pass _bwd_dist/_bwd_heading
        for the backward buffer.
        """
        if dist_buf is None:
            dist_buf = self._dist
        if heading_buf is None:
            heading_buf = self._heading

        if not dist_buf:
            return 0.0

        if target_dist <= dist_buf[0]:
            return heading_buf[0]
        if target_dist >= dist_buf[-1]:
            return heading_buf[-1]

        # Binary search for the interval containing target_dist
        idx = bisect_left(dist_buf, target_dist)

        if idx == 0:
            return heading_buf[0]
        if idx >= len(dist_buf):
            return heading_buf[-1]

        d0 = dist_buf[idx - 1]
        d1 = dist_buf[idx]
        if d1 == d0:
            return heading_buf[idx - 1]

        t = (target_dist - d0) / (d1 - d0)
        h0 = heading_buf[idx - 1]
        h1 = heading_buf[idx]
        return h0 + t * (h1 - h0)

    def kinematic(self, linear_vel, angular_vel, yaw_angle):
        """Compute next module's velocities given current module's velocities and joint angle."""
        linear_out = linear_vel * cos(yaw_angle) + self.a * angular_vel * sin(yaw_angle)
        angular_out = (
            linear_vel * sin(yaw_angle) - self.a * angular_vel * cos(yaw_angle)
        ) / self.b

        return linear_out, angular_out

    # update the latest joint states
    def joint_state_callback(self, msg):
        self.latest_feedback = msg

    # update yaw angles from the latest joint states
    def update_yaw_angles(self):
        if self.latest_feedback is None:
            return
        for i in range(self.n_mod):
            joint_name = f'mod{i + 1}__yaw_joint'
            if joint_name in self.latest_feedback.name:
                idx = self.latest_feedback.name.index(joint_name)
                self.yaw_angles[i] = self.latest_feedback.position[idx]

        # Fuse encoder feedback into path buffer to correct dead-reckoning drift
        if not hasattr(self, '_encoder_correction_applied'):
            self._encoder_correction_applied = False

        # Compute correction term: actual encoder angle - commanded angle
        # For forward motion, compare first module's encoder vs path buffer
        if self._prev_sign == 1 and len(self._dist) > 1:
            # Get expected heading from forward path buffer at current head distance
            expected_heading = self._interp_heading(self.head_distance)
            actual_heading = self.yaw_angles[0]
            
            # Compute heading error (normalized to [-π, π])
            heading_error = atan2(sin(actual_heading - expected_heading), 
                                 cos(actual_heading - expected_heading))
            
            # Apply gentle correction to path buffer (K = 0.1 to 0.3)
            K_correction = 0.2  # Correction gain
            if abs(heading_error) > 0.01:
                correction = K_correction * heading_error
                
                for i in range(len(self._heading)):
                    self._heading[i] += correction
                self.head_theta += correction
                self.get_logger().debug(
                    f'Encoder fusion: corrected path buffer by {correction:.4f} rad'
                )

        # For backward motion, compare last module's encoder vs backward path buffer
        elif self._prev_sign == -1 and len(self._bwd_dist) > 1:
            # Get expected heading from backward path buffer
            expected_heading = self._interp_heading(
                self._bwd_distance, self._bwd_dist, self._bwd_heading
            )
            # Last module is index n_mod-1 (physical leader in backward mode)
            actual_heading = self.yaw_angles[self.n_mod - 1]
            
            heading_error = atan2(sin(actual_heading - expected_heading),
                                 cos(actual_heading - expected_heading))
            
            K_correction = 0.2
            if abs(heading_error) > 0.01:
                correction = K_correction * heading_error
                for i in range(len(self._bwd_heading)):
                    self._bwd_heading[i] += correction
                self._bwd_head_theta += correction
                self.get_logger().debug(
                    f'Encoder fusion (backward): corrected by {correction:.4f} rad'
                )

    def _compute_adaptive_alpha(self, error_mag):
        # Tunable parameters
        max_alpha = 1.0
        min_alpha = 0.1
        k = 3.0  # sensitivity

        alpha = max_alpha / (1.0 + k * error_mag)
        alpha = max(min_alpha, min(max_alpha, alpha))
        return alpha


def main(args=None):
    rclpy.init(args=args)
    try:
        agevar = Agevar()
        rclpy.spin(agevar)
    except KeyboardInterrupt:
        rclpy.logging.get_logger('agevar').warn('Agevar node interrupted by user')
    except Exception as err:
        rclpy.logging.get_logger('agevar').fatal(
            f'Error in the Agevar node: {str(err)}\n{traceback.format_exc()}'
        )
    else:
        agevar.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
