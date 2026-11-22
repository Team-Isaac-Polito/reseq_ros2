#!/usr/bin/env python3
"""Republish the latest map periodically for app clients and smooth SLAM map->odom TF.

1. Map Relay (/slam_map -> /map):
   SLAM Toolbox publishes /map with transient-local durability. That is correct
   for Nav2, but rosbridge/web clients can miss the latched sample when they
   connect before the map exists. This relay keeps publishing the latest map so
   the app gets a normal fresh OccupancyGrid without requiring a reconnect.

2. SLAM TF Smoother (/slam_tf -> /tf) [IMPORTANT - DO NOT REMOVE]:
   In slam_launch.py, slam_toolbox's TransformBroadcaster is remapped via node-scoped
   rule ('slam_toolbox:/tf' -> '/slam_tf') so its raw, unfiltered step-wise map->odom
   updates (triggered every minimum_travel_distance) do not cause single-frame robot
   jitter in RViz / Nav2. This node intercepts '/slam_tf', applies a lightweight
   SE(2) exponential moving average (EMA) + per-step slew-rate clamp on (x, y, yaw),
   and publishes the smoothed map->odom transform to '/tf' at 50 Hz.
"""

import math

import rclpy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def _map_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _tf_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class MapRepublisher(Node):
    def __init__(self) -> None:
        super().__init__('map_republisher')
        self.input_topic = self.declare_parameter('input_topic', '/slam_map').value
        self.output_topic = self.declare_parameter('output_topic', '/map').value
        self.metadata_topic = self.declare_parameter('metadata_topic', '/map_metadata').value
        self.publish_rate = float(self.declare_parameter('publish_rate', 1.0).value)
        self.stamp_with_now = bool(self.declare_parameter('stamp_with_now', False).value)

        # TF smoothing parameters for map -> odom
        self.slam_tf_topic = self.declare_parameter('slam_tf_topic', '/slam_tf').value
        self.tf_alpha_pos = float(self.declare_parameter('tf_alpha_pos', 0.15).value)
        self.tf_alpha_yaw = float(self.declare_parameter('tf_alpha_yaw', 0.15).value)
        self.tf_max_step_pos = float(self.declare_parameter('tf_max_step_pos', 0.001).value)
        self.tf_max_step_yaw = float(self.declare_parameter('tf_max_step_yaw', 0.002).value)
        self.tf_snap_distance = float(self.declare_parameter('tf_snap_distance', 0.5).value)

        self._smooth_x: float | None = None
        self._smooth_y: float | None = None
        self._smooth_yaw: float | None = None

        qos = _map_qos()
        self.map_pub = self.create_publisher(OccupancyGrid, self.output_topic, qos)
        self.metadata_pub = self.create_publisher(MapMetaData, self.metadata_topic, qos)
        self.create_subscription(OccupancyGrid, self.input_topic, self._map_callback, qos)

        tf_qos = _tf_qos()
        self.tf_pub = self.create_publisher(TFMessage, '/tf', tf_qos)
        self.create_subscription(TFMessage, self.slam_tf_topic, self._slam_tf_callback, tf_qos)

        self.latest_map: OccupancyGrid | None = None
        self.timer = self.create_timer(1.0 / max(self.publish_rate, 0.1), self._publish_latest)
        self.get_logger().info(
            f'MapRepublisher ready | {self.input_topic} -> {self.output_topic} '
            f'at {self.publish_rate:.2f} Hz | TF smoother on {self.slam_tf_topic} -> /tf'
        )

    def _slam_tf_callback(self, msg: TFMessage) -> None:
        out = TFMessage()
        for t in msg.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                q = t.transform.rotation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )

                if (
                    self._smooth_x is None
                    or math.hypot(tx - self._smooth_x, ty - self._smooth_y) > self.tf_snap_distance
                ):
                    self._smooth_x = tx
                    self._smooth_y = ty
                    self._smooth_yaw = yaw
                else:
                    dx = tx - self._smooth_x
                    dy = ty - self._smooth_y
                    dyaw = math.atan2(
                        math.sin(yaw - self._smooth_yaw),
                        math.cos(yaw - self._smooth_yaw),
                    )

                    step_x = self.tf_alpha_pos * dx
                    step_y = self.tf_alpha_pos * dy
                    step_dist = math.hypot(step_x, step_y)
                    if step_dist > self.tf_max_step_pos:
                        scale = self.tf_max_step_pos / step_dist
                        step_x *= scale
                        step_y *= scale

                    step_yaw = max(
                        -self.tf_max_step_yaw,
                        min(self.tf_max_step_yaw, self.tf_alpha_yaw * dyaw),
                    )

                    self._smooth_x += step_x
                    self._smooth_y += step_y
                    self._smooth_yaw = math.atan2(
                        math.sin(self._smooth_yaw + step_yaw),
                        math.cos(self._smooth_yaw + step_yaw),
                    )

                t.transform.translation.x = self._smooth_x
                t.transform.translation.y = self._smooth_y
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = math.sin(self._smooth_yaw * 0.5)
                t.transform.rotation.w = math.cos(self._smooth_yaw * 0.5)

            out.transforms.append(t)

        if out.transforms:
            self.tf_pub.publish(out)

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        self._publish_latest()

    def _publish_latest(self) -> None:
        if self.latest_map is None:
            return

        msg = OccupancyGrid()
        msg.header = self.latest_map.header
        msg.info = self.latest_map.info
        msg.data = self.latest_map.data

        if self.stamp_with_now:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.info.map_load_time = msg.header.stamp
        elif msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            msg.header.stamp = self.get_clock().now().to_msg()

        self.map_pub.publish(msg)
        self.metadata_pub.publish(msg.info)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
