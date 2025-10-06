import traceback

import cv2
import rclpy
import rclpy.logging
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
from ultralytics import YOLO

from reseq_interfaces.msg import Detection
from reseq_ros2.reseq_cv.motion import MotionDetection
from reseq_ros2.reseq_cv.orientation import OrientationDetection
from reseq_ros2.reseq_cv.qr_reader import process_qr_codes

share_folder = get_package_share_directory('reseq_ros2')
models_path = f'{share_folder}/ml-ckpt'

"""
Updated ROS2 Detector node integrated with detection_manager.

This node subscribes to RGB and depth images, runs different detection pipelines
based on the current mode broadcasted by detection_manager, and publishes detections
in raw pixel + depth format. The mapping coordinate transformation is now handled
exclusively by detection_manager.
"""

# Mode constants
MODE_OFF = 0
MODE_INITIALIZING = 1
MODE_SENSOR_CRATE = 2
MODE_MAPPING = 3


class Detector(Node):
    def __init__(self):
        super().__init__('detector')
        self.bridge = CvBridge()

        # Mode handling
        self.current_mode = MODE_OFF
        self.create_subscription(UInt8, '/detection/mode', self._on_mode_update, 10)
        self.get_logger().info('Subscribed to /detection/mode for runtime control')

        # Camera Subscriptions
        self.color_subscription = self.create_subscription(
            Image,
            '/realsense/realsense2_camera_node/color/image_raw',
            self.image_callback,
            10,
        )
        self.depth_subscription = self.create_subscription(
            Image,
            '/realsense/realsense2_camera_node/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10,
        )
        self.depth_image = None

        # Publishers
        self.model_pub = self.create_publisher(Image, '/detector/model_output', 10)
        self.detection_pub = self.create_publisher(Detection, '/object_detection/detections', 10)
        self.depth_pub = self.create_publisher(Image, '/object_detection/depthsized', 10)

        # YOLO Models
        self.img_size = (1280, 720)
        model_path1 = f'{models_path}/hazmat_detection.pt'
        model_path2 = f'{models_path}/object_detection.pt'

        self.model1 = YOLO(model_path1)  # Hazmat
        self.model2 = YOLO(model_path2)  # Objects

        # Custom CV Modules
        self.od = OrientationDetection()
        self.md = MotionDetection()

        # Camera intrinsics (default, can be overridden later)
        self.f_x = 910.3245
        self.f_y = 909.7875
        self.c_x = 648.6353
        self.c_y = 369.6105

        # State
        self.camera_frame = 'camera_depth_optical_frame'
        self.detection_counter = 1

        self.get_logger().info('Detector initialized successfully')

    # Mode Handling
    def _on_mode_update(self, msg):
        new_mode = int(msg.data)
        if new_mode != self.current_mode:
            self.get_logger().info(f'Mode changed: {self.current_mode} -> {new_mode}')
            self.current_mode = new_mode
            self._apply_mode(new_mode)

    def _apply_mode(self, mode):
        """Adjust runtime behavior when the mode changes."""
        if mode == MODE_OFF:
            self.get_logger().info('Detector: OFF - shutting down all pipelines')
        elif mode == MODE_INITIALIZING:
            self.get_logger().info(
                'Detector: INITIALIZING - loading models only (no inference yet)'
            )
        elif mode == MODE_SENSOR_CRATE:
            self.get_logger().info('Detector: SENSOR_CRATE - running QR/Hazmat/Apriltag only')
        elif mode == MODE_MAPPING:
            self.get_logger().info('Detector: MAPPING - running YOLO + others')

    # Image Callbacks
    def image_callback(self, msg):
        """
        Main callback for color image.
        Chooses pipelines to run based on current_mode.
        """
        mode = self.current_mode

        # OFF Mode: skip everything
        if mode == MODE_OFF:
            return

        # Convert ROS Image message to OpenCV
        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        # Preprocessing: orientation, motion, QR
        od_frame, _, _ = self.od.process_image(color_image)
        qr_frame = process_qr_codes(od_frame)
        md_frame = self.md.process_image(qr_frame)
        color_image = cv2.resize(md_frame, self.img_size)

        # SENSOR_CRATE and MAPPING modes both run QR/Hazmat/Apriltag
        if mode in (MODE_SENSOR_CRATE, MODE_MAPPING):
            self._process_hazmat(color_image, img, msg)
            self._process_apriltags(color_image, img, msg)

        # MAPPING mode only: add YOLO object detection
        if mode == MODE_MAPPING:
            self._process_objects(color_image, img, msg)

        # Publish annotated output
        model_output_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        self.model_pub.publish(model_output_msg)

    def depth_callback(self, msg):
        """Depth image subscription callback."""
        if self.current_mode == MODE_OFF:
            return
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # Pipeline Processing Methods
    def _process_hazmat(self, color_image, img, msg):
        """Process hazmat detections."""
        if self.depth_image is None:
            return

        results = self.model1(img)
        for result in results:
            bbox = result.boxes.xyxy
            labels = [result.names[cls.item()] for cls in result.boxes.cls.int()]
            confs = result.boxes.conf
            for i, conf in enumerate(confs):
                if conf <= 0.5:
                    continue

                x1, y1, x2, y2 = bbox[i]
                label = labels[i]
                mid_x = int((x1 + x2) / 2)
                mid_y = int((y1 + y2) / 2)
                depth_value = float(self.depth_image[mid_y, mid_x]) / 1e3  # meters

                det_msg = self._create_detection_msg(
                    msg, 'hazmat', label, conf, x1, y1, x2, y2, depth_value
                )
                self.detection_pub.publish(det_msg)

                # Debug visualization
                cv2.circle(color_image, (mid_x, mid_y), 10, (0, 0, 255), 2)
                cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)

                # Publish depth image for debugging
                depth_img = self.bridge.cv2_to_imgmsg(self.depth_image, encoding='passthrough')
                self.depth_pub.publish(depth_img)

    def _process_apriltags(self, color_image, img, msg):
        """Placeholder for Apriltag detection logic as required by the design document."""
        # Once a tag is detected, a `reseq_interfaces/Detection` message with type='apriltag' should be
        # constructed and published.
        pass

    def _process_objects(self, color_image, img, msg):
        """Process YOLO object detections (MAPPING mode only)."""
        if self.depth_image is None:
            return

        results = self.model2(img)
        for result in results:
            bbox = result.boxes.xyxy
            labels = [result.names[cls.item()] for cls in result.boxes.cls.int()]
            confs = result.boxes.conf
            for i, conf in enumerate(confs):
                if conf <= 0.5:
                    continue

                x1, y1, x2, y2 = bbox[i]
                label = labels[i]
                mid_x = int((x1 + x2) / 2)
                mid_y = int((y1 + y2) / 2)
                depth_value = float(self.depth_image[mid_y, mid_x]) / 1e3  # meters

                det_msg = self._create_detection_msg(
                    msg, 'object', label, conf, x1, y1, x2, y2, depth_value
                )
                self.detection_pub.publish(det_msg)

                # Debug visualization
                cv2.circle(color_image, (mid_x, mid_y), 10, (255, 0, 0), 2)
                cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)

    # Helper Methods
    def _create_detection_msg(self, msg, det_type, label, conf, x1, y1, x2, y2, depth_value):
        """Create a Detection message with raw pixel/depth data."""
        det = Detection()
        det.header.stamp = msg.header.stamp
        det.header.frame_id = self.camera_frame
        det.detection = self.detection_counter
        self.detection_counter += 1

        det.type = det_type
        det.name = str(label)
        det.confidence = float(conf)

        det.xmin = int(x1)
        det.ymin = int(y1)
        det.width = int(x2 - x1)
        det.height = int(y2 - y1)
        det.depth_center = depth_value
        det.camera_frame = self.camera_frame

        det.robot = 'reseq'
        det.mode = 'T'
        return det


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
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
