#!/usr/bin/env python3
"""
TOF-based obstacle height detector for ReseQ robot.

Subscribes to TOF sensor data (from 4 VL53L1X sensors on RP2040-Zero),
processes distance measurements to detect obstacle height in front of
the robot, and publishes obstacle height data for the joint lifting system.

The 4 TOF sensors are mounted on the front of the first module at fixed
vertical angles (looking from the side, XY plane):
    Sensor 0 (down):   -90° — straight down, sees ground directly below
    Sensor 1 (down45): -45° — 45° below horizontal
    Sensor 2 (center):   0° — horizontal, straight ahead
    Sensor 3 (up45):   +45° — 45° above horizontal

Each sensor provides a single distance measurement (1x1 ROI at SPAD center).
The sensors are positioned in front of chassis1, between the wheels,
in front of the arm base.

Published Topics:
    /tof/obstacle_height (std_msgs/Float32): Estimated obstacle height in meters.
        0.0 means no obstacle detected within range.
    /tof/obstacle_distance (std_msgs/Float32): Distance to nearest obstacle in meters.
    /tof/obstacle_points (sensor_msgs/PointCloud2): 3D points from TOF array for visualization.

Parameters:
    num_sensors: Number of TOF sensors in the array (default: 4)
    min_range: Minimum valid range in meters (default: 0.03)
    max_range: Maximum valid range in meters (default: 2.0)
    height_threshold: Minimum obstacle height to trigger joint lift in meters (default: 0.03)
    max_climbable_height: Maximum obstacle height the robot can climb in meters (default: 0.15)
    detection_distance: Distance ahead to start detecting obstacles in meters (default: 0.5)
    sensor_height: Height of sensor mount above ground in meters (default: 0.0609)
    mean_filter_size: Number of samples for moving average filter (default: 5)
    body_exclusion_radius: Radius to exclude body detections in meters (default: 0.3)
    imu_topic: Topic for IMU data to compensate for pitch (default: /imu1_broadcaster/imu)
    serial_port: Serial port for Pico communication (default: /dev/ttyACM0)
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, LaserScan
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener


# Fixed mounting angles for the 4 TOF sensors (radians, XY plane, looking from side)
# Index 0 = up45 (+45°), 1 = front (0°), 2 = down45 (-45°), 3 = down (-90°)
# Matches URDF sensor order: TOF1=up45, TOF2=front, TOF3=down45, TOF4=down
SENSOR_ANGLES = [math.pi / 4, 0.0, -math.pi / 4, -math.pi / 2]


class TofObstacleDetector(Node):
    def __init__(self):
        super().__init__('tof_obstacle_detector')

        # Parameters
        self.declare_parameter('num_sensors', 4)
        self.declare_parameter('min_range', 0.03)
        self.declare_parameter('max_range', 2.0)
        self.declare_parameter('height_threshold', 0.03)
        self.declare_parameter('max_climbable_height', 0.15)
        self.declare_parameter('detection_distance', 0.5)
        self.declare_parameter('sensor_height', 0.0609)
        self.declare_parameter('mean_filter_size', 5)
        self.declare_parameter('body_exclusion_radius', 0.3)
        self.declare_parameter('imu_topic', '/imu1_broadcaster/imu')
        self.declare_parameter('serial_port', '/dev/ttyACM0')

        self._num_sensors = self.get_parameter('num_sensors').value
        self._min_range = self.get_parameter('min_range').value
        self._max_range = self.get_parameter('max_range').value
        self._height_threshold = self.get_parameter('height_threshold').value
        self._max_climbable_height = self.get_parameter('max_climbable_height').value
        self._detection_distance = self.get_parameter('detection_distance').value
        self._sensor_height = self.get_parameter('sensor_height').value
        self._mean_filter_size = self.get_parameter('mean_filter_size').value
        self._body_exclusion_radius = self.get_parameter('body_exclusion_radius').value
        self._use_sim_time = self.get_parameter('use_sim_time').value
        self._use_serial = not self._use_sim_time

        # State
        self._distances = None  # (num_sensors,) array of raw distances in meters
        self._filtered_distances = None
        self._distance_buffer = []
        self._robot_pitch = 0.0
        self._obstacle_height = 0.0
        self._obstacle_distance = float('inf')

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Subscriptions
        if self._use_serial:
            self._serial = None
            self._setup_serial()
        else:
            for i, name in enumerate(['up45', 'front', 'down45', 'down']):
                self.create_subscription(
                    LaserScan,
                    f'/tof/{name}',
                    lambda msg, idx=i: self._gazebo_scan_callback(msg, idx),
                    sensor_qos,
                )

        # IMU subscription for pitch compensation
        self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').value,
            self._imu_callback,
            sensor_qos,
        )

        # Publishers
        self._height_pub = self.create_publisher(Float32, '/tof/obstacle_height', 10)
        self._distance_pub = self.create_publisher(Float32, '/tof/obstacle_distance', 10)
        self._points_pub = self.create_publisher(PointCloud2, '/tof/obstacle_points', sensor_qos)

        # Timer for processing
        self._timer = self.create_timer(0.05, self._process)

        self.get_logger().info(
            f'TofObstacleDetector ready | sensors={self._num_sensors} | '
            f'serial={self._use_serial} | height_threshold={self._height_threshold}m | '
            f'max_climbable={self._max_climbable_height}m | '
            f'angles={[f"{a*180/math.pi:.0f}°" for a in SENSOR_ANGLES]}'
        )

    def _setup_serial(self):
        try:
            import serial
            port = self.get_parameter('serial_port').value
            self._serial = serial.Serial(port, 115200, timeout=0.1)
            self.get_logger().info(f'Serial connected to {port}')
        except Exception as e:
            self.get_logger().warn(f'Serial not available: {e}.')
            self._serial = None

    def _read_serial(self):
        """Read TOF distances from Pico via serial.
        Protocol: header (4 bytes: 0xAA, 0x55, 0xAA, 0x55) + num_sensors * 1 * 2 bytes (int16 mm).
        """
        if self._serial is None:
            return None
        try:
            header = bytes([0xAA, 0x55, 0xAA, 0x55])
            buf = bytes()
            timeout = self.get_clock().now() + rclpy.duration.Duration(seconds=0.05)
            while self.get_clock().now() < timeout:
                if self._serial.in_waiting > 0:
                    buf += self._serial.read(1)
                    if len(buf) >= 4 and buf[-4:] == header:
                        data_size = self._num_sensors * 1 * 2
                        data = self._serial.read(data_size)
                        if len(data) == data_size:
                            arr = np.frombuffer(data, dtype=np.int16)
                            return arr.astype(np.float64) * 0.001  # mm to meters
                        break
                    if len(buf) > 100:
                        buf = buf[-4:]
            return None
        except Exception as e:
            self.get_logger().debug(f'Serial read error: {e}')
            return None

    def _gazebo_scan_callback(self, msg: LaserScan, sensor_idx: int):
        if self._distances is None:
            self._distances = np.full(self._num_sensors, np.nan)
        try:
            ranges = np.array(msg.ranges)
            valid = [float(r) for r in ranges
                     if not math.isnan(r) and self._min_range <= r <= self._max_range]
            if valid:
                self._distances[sensor_idx] = min(valid)
        except Exception as e:
            self.get_logger().debug(f'Gazebo scan callback error: {e}')

    def _imu_callback(self, msg: Imu):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        self._robot_pitch = math.asin(max(-1.0, min(1.0, sinp)))

    def _process(self):
        if self._use_serial and self._serial is not None:
            serial_data = self._read_serial()
            if serial_data is not None:
                self._distances = serial_data

        if self._distances is None:
            return

        self._distance_buffer.append(self._distances.copy())
        if len(self._distance_buffer) > self._mean_filter_size:
            self._distance_buffer.pop(0)
        self._filtered_distances = np.nanmean(self._distance_buffer, axis=0)

        self._compute_obstacle_height()

        height_msg = Float32()
        height_msg.data = self._obstacle_height
        self._height_pub.publish(height_msg)

        dist_msg = Float32()
        dist_msg.data = self._obstacle_distance if self._obstacle_distance < float('inf') else 0.0
        self._distance_pub.publish(dist_msg)

    def _compute_obstacle_height(self):
        """Estimate obstacle height from 4 TOF sensors at known angles."""
        if self._filtered_distances is None:
            return

        min_obstacle_height = 0.0
        min_obstacle_dist = float('inf')

        for sensor_idx in range(self._num_sensors):
            measured = self._filtered_distances[sensor_idx]
            if math.isnan(measured) or measured < self._min_range or measured > self._max_range:
                continue

            sensor_angle = SENSOR_ANGLES[sensor_idx]
            effective_angle = sensor_angle + self._robot_pitch

            if abs(effective_angle) < 0.01:
                continue

            # Expected distance to flat ground: h = d * sin(angle) => d = h / sin(angle)
            expected_ground_dist = self._sensor_height / math.sin(abs(effective_angle))

            if measured < expected_ground_dist * 0.9:
                # Obstacle height: h_obstacle = h_sensor - measured * sin(angle)
                obstacle_h = self._sensor_height - measured * math.sin(abs(effective_angle))
                obstacle_h = max(0.0, obstacle_h)
                horizontal_dist = measured * math.cos(effective_angle)

                if obstacle_h > self._height_threshold:
                    if horizontal_dist > self._body_exclusion_radius:
                        if obstacle_h > min_obstacle_height:
                            min_obstacle_height = obstacle_h
                            min_obstacle_dist = horizontal_dist

        self._obstacle_height = min_obstacle_height
        self._obstacle_distance = min_obstacle_dist


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TofObstacleDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback as tb
        print(f'Error in TofObstacleDetector: {e}\n{tb.format_exc()}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
