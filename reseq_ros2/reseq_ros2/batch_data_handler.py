#!/usr/bin/env python3

from datetime import datetime

import pandas as pd
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

from reseq_interfaces.srv import BatchDetections2D


class BatchDataHandler(Node):
    def __init__(self):
        super().__init__('batch_data_handler')
        # Parameters for CSV header metadata
        self.declare_parameter('team_name', 'YourTeam')
        self.declare_parameter('country', 'YourCountry')
        self.declare_parameter('mission', 1)

        # Create the batch-reporting service
        self.srv = self.create_service(
            BatchDetections2D, 'batch_detections_2d', self.batch_callback
        )

        self.get_logger().info('BatchDataHandler ready.')

    def batch_callback(self, req, res):
        # Build the CSV header lines
        now = datetime.now()
        header_lines = [
            'pois',
            '1.3',
            self.get_parameter('team_name').value,
            self.get_parameter('country').value,
            now.strftime('%Y-%m-%d'),
            now.strftime('%H:%M:%S'),
            str(self.get_parameter('mission').value),
        ]

        # Unpack detections into rows
        rows = []
        for idx, det2d in enumerate(req.detections.detections, start=1):
            hyp = det2d.results[0]  # take the first hypothesis
            x = det2d.bbox.center.position.x
            y = det2d.bbox.center.position.y
            z = 0.0  # 2D detection has no depth

            # Get name from encoded_data (this is the dynamic content)
            name = req.encoded_data[idx - 1] if len(req.encoded_data) >= idx else ''

            rows.append(
                {
                    'detection': idx,
                    'time': det2d.header.stamp.sec + det2d.header.stamp.nanosec * 1e-9,
                    'type': hyp.hypothesis.class_id,  # Type of the object
                    'name': name,  # The unique identifier (example.com for QR code, etc.)
                    'x': x,
                    'y': y,
                    'z': z,
                    'robot': req.robot,
                    'mode': req.mode,
                    'confidence': hyp.hypothesis.score,
                }
            )

        # Write to CSV (append mode)
        df = pd.DataFrame(rows)
        filename = f'{req.robot}_detections.csv'
        with open(filename, 'a') as f:
            if f.tell() == 0:  # Check if the file is empty, if so write header
                for line in header_lines:
                    f.write(f'"{line}"\n')
            df.to_csv(f, index=False, header=False)

        self.get_logger().info(f'Wrote {len(rows)} detections to {filename}')
        res.success = True
        res.message = filename
        return res


def main(args=None):
    rclpy.init(args=args)
    node = BatchDataHandler()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
