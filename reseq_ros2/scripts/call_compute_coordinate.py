#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from reseq_interfaces.msg import Detection
from reseq_interfaces.srv import ComputeCoordinate


class ComputeClient(Node):
    def __init__(self):
        super().__init__('compute_client')
        self.cli = self.create_client(ComputeCoordinate, '/detection/compute_coordinate')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /detection/compute_coordinate service...')
        self.req = ComputeCoordinate.Request()

    def call_service(self):
        det = Detection()
        det.header.stamp = self.get_clock().now().to_msg()
        det.header.frame_id = 'camera_depth_optical_frame'
        det.detection = 999
        det.type = 'object'
        det.name = 'test_object'
        det.robot = 'reseq'
        det.mode = 'T'
        det.confidence = 0.95
        det.xmin = 600
        det.ymin = 340
        det.width = 100
        det.height = 120
        det.depth_center = 1.5
        det.camera_frame = 'camera_depth_optical_frame'

        self.req.detection = det
        self.req.target_frame = 'odom'

        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            res = future.result()
            if res.success:
                p = res.point
                self.get_logger().info(
                    f'Point in {p.header.frame_id}: '
                    f'x={p.point.x:.2f}, y={p.point.y:.2f}, z={p.point.z:.2f}'
                )
            else:
                self.get_logger().error(f'Failed: {res.message}')
        else:
            self.get_logger().error('Service call failed!')


def main(args=None):
    rclpy.init(args=args)
    node = ComputeClient()
    node.call_service()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
