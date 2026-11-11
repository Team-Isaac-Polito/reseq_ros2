"""Thin out the depth camera point cloud before Nav2 reads it.

The full 640x480 cloud is about 7 MB. Over DDS it splits into thousands of fragments,
and best-effort subscribers drop the whole cloud when one fragment is lost, so the
collision monitor saw a new cloud only every few seconds and kept stopping the robot.
One subscriber here keeps every Nth pixel and only x, y and z.
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def decimate(message, stride):
    """Return an (N, 3) float32 array from every stride-th row and column."""
    offsets = {}
    for field in message.fields:
        offsets[field.name] = field.offset

    # View the raw bytes as image rows, and keep every stride-th row.
    data = np.frombuffer(bytes(message.data), dtype=np.uint8)
    rows = data.reshape(message.height, message.row_step)
    rows = rows[::stride]

    # Rows may have padding at the end; cut it off, then split rows into points.
    row_bytes = message.width * message.point_step
    rows = rows[:, :row_bytes]
    points = rows.reshape(len(rows), message.width, message.point_step)

    # Keep every stride-th point in each row, then make one flat list of points.
    points = points[:, ::stride]
    points = points.reshape(-1, message.point_step)

    # Copy the 4 bytes of x, y and z out of each point and read them as floats.
    xyz = np.empty((len(points), 3), dtype=np.float32)
    for column, name in enumerate('xyz'):
        start = offsets[name]
        field_bytes = points[:, start:start + 4].copy()
        xyz[:, column] = field_bytes.view(np.float32).ravel()

    # Drop points without a depth reading (NaN or inf).
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid]


class CloudDecimator(Node):
    def __init__(self):
        super().__init__('cloud_decimator')
        input_topic = self.declare_parameter('input_topic', '/realsense/depth/color/points').value
        output_topic = self.declare_parameter('output_topic', '/traction/depth/points').value
        self.stride = int(self.declare_parameter('stride', 8).value)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.output = self.create_publisher(PointCloud2, output_topic, sensor_qos)
        self.create_subscription(PointCloud2, input_topic, self._cloud_cb, sensor_qos)

    def _cloud_cb(self, message):
        if message.is_bigendian:
            self.get_logger().warning('Big-endian clouds are not supported', once=True)
            return
        xyz = decimate(message, self.stride)
        cloud = point_cloud2.create_cloud_xyz32(message.header, xyz)
        self.output.publish(cloud)


def main():
    rclpy.init()
    node = CloudDecimator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
