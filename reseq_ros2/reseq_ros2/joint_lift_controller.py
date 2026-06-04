#!/usr/bin/env python3
"""
Joint lift controller for ReseQ robot obstacle climbing.

Subscribes to TOF obstacle height data and IMU readings, then publishes
joint lift commands to raise the front modules when an obstacle is detected
that is within the climbable height range.

This node works in conjunction with the Agevar node. It modifies the
follow-the-leader behavior by injecting additional pitch commands to lift
the front modules when approaching an obstacle.

The lifting strategy:
1. When an obstacle is detected within detection_distance and its height
   is between height_threshold and max_climbable_height:
    a. Gradually increase the pitch angle of the front joint(s) to lift the
      front module over the obstacle.
   b. The lift amount is proportional to the obstacle height.
   c. Once the front module is over the obstacle, gradually lower it.
2. The lift is smoothed to avoid abrupt joint movements.
3. IMU data is used to compensate for robot pitch on slopes.

Published Topics:
    /joint_lift/commands (std_msgs/Float64MultiArray): Additional pitch joint commands
        to be added to the Agevar output. Size = n_joints, values in radians.
    /joint_lift/status (std_msgs/String): Current lift state for debugging.

Parameters:
    max_climbable_height: Maximum obstacle height the robot can climb (default: 0.15 m)
    lift_angle_per_meter: Pitch angle increase per meter of obstacle height (default: 2.0 rad/m)
    approach_distance: Distance before obstacle to start lifting (default: 0.4 m)
    lift_duration: Time to complete the lift maneuver in seconds (default: 2.0)
    smoothing_factor: Exponential smoothing for lift commands (default: 0.3)
    pitch_compensation_gain: Gain for IMU pitch compensation (default: 0.5)
    body_filter_enabled: Enable body-ghost filtering (default: True)
    min_obstacle_width: Minimum obstacle width to trigger lift in meters (default: 0.1)
"""

import math
import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, Float64MultiArray, String


class JointLiftController(Node):
    def __init__(self):
        super().__init__('joint_lift_controller')

        # Parameters
        self.declare_parameter('max_climbable_height', 0.15)
        self.declare_parameter('lift_angle_per_meter', 2.0)
        self.declare_parameter('approach_distance', 0.4)
        self.declare_parameter('lift_duration', 2.0)
        self.declare_parameter('smoothing_factor', 0.3)
        self.declare_parameter('pitch_compensation_gain', 0.5)
        self.declare_parameter('body_filter_enabled', True)
        self.declare_parameter('min_obstacle_width', 0.1)
        self.declare_parameter('imu_topic', '/imu1_broadcaster/imu')
        self.declare_parameter('scan_topic', '/scan')

        self._max_climbable_height = self.get_parameter('max_climbable_height').value
        self._lift_angle_per_meter = self.get_parameter('lift_angle_per_meter').value
        self._approach_distance = self.get_parameter('approach_distance').value
        self._lift_duration = self.get_parameter('lift_duration').value
        self._smoothing_factor = self.get_parameter('smoothing_factor').value
        self._pitch_compensation_gain = self.get_parameter('pitch_compensation_gain').value
        self._body_filter_enabled = self.get_parameter('body_filter_enabled').value
        self._min_obstacle_width = self.get_parameter('min_obstacle_width').value

        # State
        self._obstacle_height = 0.0
        self._obstacle_distance = float('inf')
        self._robot_pitch = 0.0
        self._lift_active = False
        self._lift_phase = 'idle'  # idle, approaching, lifting, holding, lowering
        self._lift_progress = 0.0  # 0..1
        self._current_lift_command = None  # smoothed lift values per joint
        self._n_joints = 0  # will be determined from first command
        self._last_scan = None

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Subscriptions
        self.create_subscription(Float32, '/tof/obstacle_height', self._height_cb, sensor_qos)
        self.create_subscription(Float32, '/tof/obstacle_distance', self._distance_cb, sensor_qos)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self._imu_cb, sensor_qos
        )

        # Body-ghost filter: subscribe to LiDAR scan to detect and exclude body points
        if self._body_filter_enabled:
            self.create_subscription(
                LaserScan,
                self.get_parameter('scan_topic').value,
                self._scan_cb,
                sensor_qos,
            )

        # Publishers
        self._cmd_pub = self.create_publisher(Float64MultiArray, '/joint_lift/commands', 10)
        self._status_pub = self.create_publisher(String, '/joint_lift/status', 10)

        # Timer
        self._timer = self.create_timer(0.05, self._update)  # 20 Hz

        self.get_logger().info(
            f'JointLiftController ready | max_height={self._max_climbable_height}m | '
            f'approach_dist={self._approach_distance}m | body_filter={self._body_filter_enabled}'
        )

    def _height_cb(self, msg: Float32):
        self._obstacle_height = msg.data

    def _distance_cb(self, msg: Float32):
        self._obstacle_distance = msg.data if msg.data > 0 else float('inf')

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        self._robot_pitch = math.asin(max(-1.0, min(1.0, sinp)))

    def _scan_cb(self, msg: LaserScan):
        """Store latest LiDAR scan for body-ghost filtering."""
        self._last_scan = msg

    def _is_body_ghost(self, obstacle_dist: float) -> bool:
        """Check if a detected obstacle is likely the robot's own body.

        Uses LiDAR scan data to detect if there are scan points very close
        to the robot that could be the body. If the obstacle distance matches
        the body extent, it's likely a ghost.

        Also uses IMU pitch to detect if the robot is on a slope (which could
        cause the LiDAR to see the ground as an obstacle).
        """
        if self._last_scan is None:
            return False

        # Check if robot is on a steep slope (LiDAR might see ground)
        if abs(self._robot_pitch) > 0.3:  # > ~17 degrees
            # On steep slopes, close obstacles might be the ground
            if obstacle_dist < 0.5:
                return True

        # Check LiDAR for body-like close points
        ranges = np.array(self._last_scan.ranges)
        angles = np.linspace(
            self._last_scan.angle_min,
            self._last_scan.angle_max,
            len(ranges),
        )

        # Points very close to the robot (within body extent)
        body_mask = (ranges > 0.05) & (ranges < 0.35)
        if np.sum(body_mask) > 5:
            # There are many close points — likely the body
            # Check if obstacle distance matches body extent
            if obstacle_dist < 0.4:
                return True

        return False

    def _update(self):
        """Main update loop: compute lift commands based on obstacle data."""
        msg = Float64MultiArray()
        status = String()

        # Check if obstacle is valid and within range
        if (
            self._obstacle_height > 0
            and self._obstacle_distance < self._approach_distance * 3
            and self._obstacle_height <= self._max_climbable_height
        ):
            # Body-ghost filter
            if self._body_filter_enabled and self._is_body_ghost(self._obstacle_distance):
                status.data = 'filtered:body_ghost'
                self._status_pub.publish(status)
                msg.data = []
                self._cmd_pub.publish(msg)
                return

            # Compute target lift angle based on obstacle height
            target_lift = self._obstacle_height * self._lift_angle_per_meter

            # Compensate for robot pitch (on slopes, need more/less lift)
            pitch_compensation = self._robot_pitch * self._pitch_compensation_gain
            target_lift += pitch_compensation
            target_lift = max(0.0, target_lift)

            # Determine lift phase based on distance
            if self._obstacle_distance > self._approach_distance:
                self._lift_phase = 'approaching'
                self._lift_progress = max(
                    0.0, 1.0 - (self._obstacle_distance / self._approach_distance)
                )
            elif self._obstacle_distance > 0.1:
                self._lift_phase = 'lifting'
                self._lift_progress = 1.0
            else:
                self._lift_phase = 'lowering'
                self._lift_progress = max(0.0, self._obstacle_distance / 0.1)

            # Smooth the lift command
            if self._current_lift_command is None:
                self._current_lift_command = 0.0

            self._current_lift_command = (
                self._smoothing_factor * target_lift * self._lift_progress
                + (1.0 - self._smoothing_factor) * self._current_lift_command
            )

            # Build per-joint lift command
            # Front joints get more lift, rear joints get less
            # This creates a gradual "wave" that lifts the robot over the obstacle
            lift_value = self._current_lift_command

            status.data = (
                f'phase={self._lift_phase} '
                f'height={self._obstacle_height:.3f}m '
                f'dist={self._obstacle_distance:.2f}m '
                f'lift={lift_value:.3f}rad '
                f'pitch={self._robot_pitch:.2f}rad'
            )

        else:
            # No obstacle — gradually lower
            self._lift_phase = 'idle'
            self._lift_progress = 0.0
            if self._current_lift_command is not None:
                self._current_lift_command *= 0.9  # decay
                if self._current_lift_command < 0.01:
                    self._current_lift_command = 0.0
            lift_value = 0.0
            status.data = 'idle'

        self._status_pub.publish(status)
        msg.data = [float(lift_value)]
        self._cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = JointLiftController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback as tb

        print(f'Error in JointLiftController: {e}\n{tb.format_exc()}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
