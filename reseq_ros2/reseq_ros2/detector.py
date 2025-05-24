import sys
import os
import traceback

import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

# Add `/src` to PYTHONPATH to import the necessary modules from reseq_cv
sys.path.append(os.path.expanduser('~/ros2_ws/src'))

from reseq_cv.orientation_detection.concentric_c import OrientationDetection
from reseq_cv.motion_detection.motion_detection import MotionDetection
from reseq_cv.qr_apriltag_detection.qr_reader import process_qr_codes
from reseq_interfaces.msg import Detection
# from reseq_cv.qr_apriltag_detection.apriltag_reader import process_apriltags  # not available

"""
ROS node for detection and processing images using YOLO and custom models.

This node subscribes to a camera image topic, processes the incoming images
using YOLO models for object and hazmat detection, and publishes the processed images
to a specified topic. It also integrates additional detection models for orientation detection,
motion detection, qr code detection, and apriltag detection.
"""


class Detector(Node):
    def __init__(self):
        super().__init__('detector')
        self.bridge = CvBridge()

        # Subscribe to the camera color and depth image topic
        self.color_subscription = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10
        )
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10,
        )
        self.depth_image = None

        # Create publishers for the model output
        self.model_pub = self.create_publisher(Image, '/detector/model_output', 10)
        self.detection_pub = self.create_publisher(Detection, '/object_detection/detections', 10)
        self.depth_pub = self.create_publisher(Image, '/object_detection/depthsized', 10)

        # Set the image size for the YOLO model
        self.img_size = (1280, 720)

        # Define the model path
        model_path1 = os.path.expanduser('~/ros2_ws/src/reseq_cv/hazmat_detection/best.pt')
        model_path2 = os.path.expanduser('~/ros2_ws/src/reseq_cv/object_detection/best.pt')

        # Initialize the YOLO model
        self.model1 = YOLO(model_path1)
        self.model2 = YOLO(model_path2)

        # Initialize the orientation and motion detection classes
        self.od = OrientationDetection()
        self.md = MotionDetection()

        # Take parameters from /camera/camera/aligned_depth_to_color/camera_info
        self.f_x = 910.3245
        self.f_y = 909.7875
        self.c_x = 648.6353
        self.c_y = 369.6105

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert color image to tensor
        img = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        od_frame, _, _ = self.od.process_image(color_image)
        qr_frame = process_qr_codes(od_frame)
        # at_frame = process_apriltags(qr_frame)
        md_frame = self.md.process_image(qr_frame)
        color_image = cv2.resize(md_frame, (self.img_size[0], self.img_size[1]))

        # Predict hazmat
        results = self.model1(img)

        # Process results
        for result in results:
            bbox = result.boxes.xyxy
            labels = [result.names[cls.item()] for cls in result.boxes.cls.int()]
            confs = result.boxes.conf
            for i, conf in enumerate(confs):
                if conf > 0.5:  # Confidence threshold
                    x1, y1, x2, y2 = bbox[i]
                    label = labels[i]

                    # Topic for CSV file
                    if self.depth_image is not None:
                        mid_x = int((x1 + x2) / 2)
                        mid_y = int((y1 + y2) / 2)
                        detection_msg = Detection()
                        detection_msg.detection = getattr(self, 'detection_counter', 1)
                        self.detection_counter = detection_msg.detection + 1
                        detection_msg.time = self.get_clock().now().to_msg()
                        detection_msg.type = 'hazmat_sign'
                        detection_msg.name = str(label)
                        detection_msg.z = float(self.depth_image[mid_y, mid_x]) / 1e3
                        detection_msg.x = float((mid_x - self.c_x) * detection_msg.z / self.f_x)
                        detection_msg.y = float((mid_y - self.c_y) * detection_msg.z / self.f_y)
                        cv2.circle(
                            color_image,
                            (mid_x, mid_y),
                            20,
                            (0, 0, 255),
                            10,
                        )
                        cv2.circle(
                            self.depth_image,
                            (mid_x, mid_y),
                            20,
                            (0, 0, 255),
                            10,
                        )
                        depth_image = self.bridge.cv2_to_imgmsg(
                            self.depth_image, encoding='passthrough'
                        )
                        self.depth_pub.publish(depth_image)
                        detection_msg.robot = 'reseq'
                        detection_msg.mode = 'A'
                        detection_msg.confidence = float(conf)
                        self.detection_pub.publish(detection_msg)

                    # Topic for image analysis
                    cv2.rectangle(
                        color_image,
                        (int(x1), int(y1)),
                        (int(x2), int(y2)),
                        (255, 0, 0),
                        2,
                    )
                    cv2.putText(
                        color_image,
                        f'{label}: {conf:.2f}',
                        (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,
                        (255, 0, 0),
                        2,
                    )

        # Predict object
        results = self.model2(img)

        # Process results
        for result in results:
            bbox = result.boxes.xyxy
            labels = [result.names[cls.item()] for cls in result.boxes.cls.int()]
            confs = result.boxes.conf
            for i, conf in enumerate(confs):
                if conf > 0.5:  # Confidence threshold
                    x1, y1, x2, y2 = bbox[i]
                    label = labels[i]

                    # Topic for CSV file
                    if self.depth_image is not None:
                        mid_x = int((x1 + x2) / 2)
                        mid_y = int((y1 + y2) / 2)
                        detection_msg = Detection()
                        detection_msg.detection = getattr(self, 'detection_counter', 1)
                        self.detection_counter = detection_msg.detection + 1
                        detection_msg.time = self.get_clock().now().to_msg()
                        detection_msg.type = 'real_object'
                        detection_msg.name = str(label)
                        detection_msg.z = float(self.depth_image[mid_y, mid_x]) / 1e3
                        detection_msg.x = float((mid_x - self.c_x) * detection_msg.z / self.f_x)
                        detection_msg.y = float((mid_y - self.c_y) * detection_msg.z / self.f_y)
                        detection_msg.robot = 'reseq'
                        detection_msg.mode = 'A'
                        detection_msg.confidence = float(conf)
                        self.detection_pub.publish(detection_msg)

                    # Topic for image analysis
                    cv2.rectangle(
                        color_image,
                        (int(x1), int(y1)),
                        (int(x2), int(y2)),
                        (255, 0, 0),
                        2,
                    )
                    cv2.putText(
                        color_image,
                        f'{label}: {conf:.2f}',
                        (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,
                        (255, 0, 0),
                        2,
                    )

        # Convert OpenCV image back to ROS Image message
        model_output_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')

        # Publish the model output
        self.model_pub.publish(model_output_msg)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()
            cv2.destroyAllWindows()

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


def main(args=None):
    rclpy.init(args=args)
    try:
        detector = Detector()
        rclpy.spin(detector)
    except Exception as err:
        rclpy.logging.get_logger('detector').fatal(
            f'Error in the Detector node: {str(err)}\n{traceback.format_exc()}'
        )
        raise err
    else:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
