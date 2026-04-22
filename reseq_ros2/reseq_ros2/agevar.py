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

        # Smoothed yaw commands per joint (stores URDF joint angles)
        self.yaw_commands = [0.0] * self.n_joints
        self.smooth_alpha = 1.0

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
                pub.publish(TwistStamped())  # stop all controllers
            for pub in self.yaw_pubs:
                msg = Float64MultiArray()
                msg.data = [0.0]
                pub.publish(msg)
            self.yaw_commands = [0.0] * self.n_joints

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

        # Detect sim clock stuck at 0 (common during startup)
        if dt <= 0 or (not self.clock_valid and now.nanoseconds == 0):
            # Use wall clock as fallback (assumes ~real-time velocity interpretation)
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

        dt = self._compute_dt()
        if dt <= 0:
            return

        self.update_yaw_angles()

        abs_v = abs(linear_vel)
        abs_w = abs(angular_vel)

        is_rotating = abs_w > 0.05
        is_moving = abs_v > 0.01

        if is_rotating or is_moving:
            # Dead-reckon head heading: θ += ω * dt
            self.head_theta += angular_vel * dt

            if is_moving:
                # Normal forward/backward motion: advance path by true distance
                self.head_distance += abs_v * dt
            else:
                # Pure rotation: advance path by a small virtual distance so
                # the path buffer fills and joint angle commands are generated.
                # The virtual step is proportional to turn rate so faster turns
                # propagate angles at the right relative speed.
                virtual_v = abs_w * self.module_spacing * 2.0
                self.head_distance += virtual_v * dt

            # Store heading at this distance
            self._dist.append(self.head_distance)
            self._heading.append(self.head_theta)

            # Prune old entries no joint will ever need
            max_lookback = (self.n_joints + 1) * self.module_spacing + 0.5
            min_needed = self.head_distance - max_lookback
            while len(self._dist) > 2 and self._dist[1] < min_needed:
                self._dist.popleft()
                self._heading.popleft()

            # Compute joint angles via follow-the-leader heading differences
            for j in range(self.n_joints):
                if is_moving:
                    front_dist = self.head_distance - j * self.module_spacing
                    rear_dist = self.head_distance - (j + 1) * self.module_spacing

                    front_heading = self._interp_heading(front_dist)
                    rear_heading = self._interp_heading(rear_dist)

                    # Joint angle = heading difference (negative for left turns in URDF convention)
                    raw_angle = rear_heading - front_heading
                    # Normalize to [-π, π]
                    raw_angle = atan2(sin(raw_angle), cos(raw_angle))
                    # Clamp to joint limits
                    raw_angle = max(-YAW_LIMIT, min(YAW_LIMIT, raw_angle))
                else:
                    # Pure rotation: saturate all joints to YAW_LIMIT in turn direction
                    turn_dir = 1.0 if angular_vel > 0 else -1.0
                    raw_angle = turn_dir * YAW_LIMIT

                # Exponential smoothing
                self.yaw_commands[j] += self.smooth_alpha * (raw_angle - self.yaw_commands[j])

                # Publish to ForwardCommandController (direct URDF convention)
                yaw_msg = Float64MultiArray()
                yaw_msg.data = [self.yaw_commands[j]]
                self.yaw_pubs[j].publish(yaw_msg)

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

    def _interp_heading(self, target_dist):
        """Interpolate heading from the path buffer using binary search."""
        if not self._dist:
            return 0.0

        if target_dist <= self._dist[0]:
            return self._heading[0]
        if target_dist >= self._dist[-1]:
            return self._heading[-1]

        # Binary search for the interval containing target_dist
        # bisect_left on a deque via list conversion for the search key
        idx = bisect_left(self._dist, target_dist)

        if idx == 0:
            return self._heading[0]
        if idx >= len(self._dist):
            return self._heading[-1]

        d0 = self._dist[idx - 1]
        d1 = self._dist[idx]
        if d1 == d0:
            return self._heading[idx - 1]

        t = (target_dist - d0) / (d1 - d0)
        h0 = self._heading[idx - 1]
        h1 = self._heading[idx]
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
