#!/usr/bin/env python3
"""Republish the latest map periodically for app clients.

SLAM Toolbox publishes /map with transient-local durability. That is correct
for Nav2, but rosbridge/web clients can miss the latched sample when they
connect before the map exists. This relay keeps publishing the latest map so
the app gets a normal fresh OccupancyGrid without requiring a reconnect.
"""

import rclpy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _map_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class MapRepublisher(Node):
    def __init__(self) -> None:
        super().__init__('map_republisher')
        self.input_topic = self.declare_parameter('input_topic', '/slam_map').value
        self.output_topic = self.declare_parameter('output_topic', '/map').value
        self.metadata_topic = self.declare_parameter('metadata_topic', '/map_metadata').value
        self.publish_rate = float(self.declare_parameter('publish_rate', 1.0).value)
        self.stamp_with_now = bool(self.declare_parameter('stamp_with_now', False).value)

        qos = _map_qos()
        self.map_pub = self.create_publisher(OccupancyGrid, self.output_topic, qos)
        self.metadata_pub = self.create_publisher(MapMetaData, self.metadata_topic, qos)
        self.create_subscription(OccupancyGrid, self.input_topic, self._map_callback, qos)

        self.latest_map: OccupancyGrid | None = None
        self.timer = self.create_timer(1.0 / max(self.publish_rate, 0.1), self._publish_latest)
        self.get_logger().info(
            f'MapRepublisher ready | {self.input_topic} -> {self.output_topic} '
            f'at {self.publish_rate:.2f} Hz'
        )

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
