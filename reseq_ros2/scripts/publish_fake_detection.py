#!/usr/bin/env python3
import argparse
import rclpy
from rclpy.node import Node
from reseq_interfaces.msg import Detection
from std_msgs.msg import Header


class FakeDetectionPublisher(Node):
    def __init__(self, det_type, det_name):
        super().__init__('fake_detection_publisher')
        self.publisher_ = self.create_publisher(Detection, '/object_detection/detections', 10)
        self.timer = self.create_timer(1.0, self.publish_detection)
        self.det_type = det_type
        self.det_name = det_name
        self.get_logger().info(
            f"Publishing fake detection with type='{self.det_type}' and name='{self.det_name}'"
        )

    def publish_detection(self):
        msg = Detection()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'

        msg.type = self.det_type
        msg.name = self.det_name

        msg.confidence = 0.99
        msg.xmin = 100
        msg.ymin = 100
        msg.width = 50
        msg.height = 50
        msg.depth_center = 1.5

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: type="{msg.type}", name="{msg.name}"')
        # Stop after one message
        self.destroy_timer(self.timer)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Publish a fake ROS2 detection message.')
    parser.add_argument(
        '--type',
        type=str,
        required=True,
        help='The type of the detection (e.g., qr, hazmat, apriltag)',
    )
    parser.add_argument(
        '--name',
        type=str,
        required=True,
        help='The name/payload of the detection (e.g., "Hello World", "explosive", "ID42")',
    )

    # ROS2 nodes remove ROS-specific arguments automatically.
    # We parse the known arguments.
    parsed_args, _ = parser.parse_known_args()

    node = FakeDetectionPublisher(det_type=parsed_args.type, det_name=parsed_args.name)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
