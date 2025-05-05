import pandas as pd
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from reseq_interfaces.msg import Detection


class DetectionDataHandler(Node):
    def __init__(self):
        super().__init__('detection_data_handler')

        # parameter for CSV output path
        self.declare_parameter('output_path', 'detections.csv')

        # buffer for incoming detections
        self.detections = []

        # subscribe to the detection topic
        self.create_subscription(
            Detection, '/object_detection/detections', self.detection_callback, 10
        )

        # service to save buffered detections to CSV
        self.create_service(Trigger, '/object_detection/save_data', self.save_callback)

        self.get_logger().info('DetectionDataHandler node started.')

    def detection_callback(self, msg: Detection):
        # store each detection as dict
        self.detections.append(
            {
                'detection': msg.detection,
                'time': msg.time.sec + msg.time.nanosec * 1e-9,
                'type': msg.type,
                'name': msg.name,
                'x': msg.x,
                'y': msg.y,
                'z': msg.z,
                'robot': msg.robot,
                'mode': msg.mode,
                'confidence': msg.confidence,
            }
        )

    def save_callback(self, request, response):
        # get path parameter
        path = self.get_parameter('output_path').value

        # write to CSV using pandas
        df = pd.DataFrame(self.detections)
        df.to_csv(path, index=False)
        self.get_logger().info(f'Wrote {len(self.detections)} detections to {path}')
        response.success = True
        response.message = f'File saved to: {path}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DetectionDataHandler()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
