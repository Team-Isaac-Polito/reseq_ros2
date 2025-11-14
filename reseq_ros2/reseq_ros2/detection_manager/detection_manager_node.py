import os
import threading
import traceback

import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, UInt8

from reseq_interfaces.msg import Detection
from reseq_interfaces.srv import ComputeCoordinate, GetStatus, SetMode

from .compute_coordinate import compute_coordinate
from .csv_writer import CSVWriter

# Mode constants
MODE_OFF = 0
MODE_INITIALIZING = 1
MODE_SENSOR_CRATE = 2
MODE_MAPPING = 3

DEFAULT_CSV_HEADER = [
    'timestamp_iso',
    'detection_id',
    'type',
    'name',
    'confidence',
    'robot',
    'detection_mode',
    'camera_frame',
    'target_frame',
    'x_target',
    'y_target',
    'z_target',
    'u_center',
    'v_center',
    'depth_m',
    'bbox_xmin',
    'bbox_ymin',
    'bbox_w',
    'bbox_h',
]


class DetectionManager(Node):
    def __init__(self):
        super().__init__('detection_manager')
        # parameters
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('csv_default_path', os.path.expanduser('~/reseq_detections.csv'))
        self.declare_parameter('tf_timeout_sec', 0.5)
        self.declare_parameter('f_x', 910.3245)
        self.declare_parameter('f_y', 909.7875)
        self.declare_parameter('c_x', 648.6353)
        self.declare_parameter('c_y', 369.6105)
        self.declare_parameter('robot', 'reseq')
        self.declare_parameter('csv_fsync', True)
        self.declare_parameter('csv_background', False)

        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.csv_default_path = (
            self.get_parameter('csv_default_path').get_parameter_value().string_value
        )
        self.tf_timeout_sec = (
            self.get_parameter('tf_timeout_sec').get_parameter_value().double_value
        )
        self.fallback_intrinsics = {
            'f_x': self.get_parameter('f_x').get_parameter_value().double_value,
            'f_y': self.get_parameter('f_y').get_parameter_value().double_value,
            'c_x': self.get_parameter('c_x').get_parameter_value().double_value,
            'c_y': self.get_parameter('c_y').get_parameter_value().double_value,
        }
        self.robot = self.get_parameter('robot').get_parameter_value().string_value
        self.csv_fsync = self.get_parameter('csv_fsync').get_parameter_value().bool_value
        self.csv_background = self.get_parameter('csv_background').get_parameter_value().bool_value

        # state
        self.mode = MODE_OFF
        self.mode_lock = threading.Lock()
        self.initialized = False
        self.csv_path = self.csv_default_path
        self.csv_writer = None
        self.last_error = ''

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS
        mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        det_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST
        )
        text_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # publishers
        self.pub_mode = self.create_publisher(UInt8, '/detection/mode', qos_profile=mode_qos)
        self.pub_qr = self.create_publisher(String, '/qr_text', qos_profile=text_qos)
        self.pub_hazmat = self.create_publisher(String, '/hazmat_text', qos_profile=text_qos)
        self.pub_apriltag = self.create_publisher(String, '/apriltag_text', qos_profile=text_qos)

        # subscriptions
        self.sub_detections = self.create_subscription(
            Detection, '/object_detection/detections', self._on_detection, qos_profile=det_qos
        )

        # services
        self.srv_set_mode = self.create_service(
            SetMode, '/detection/set_mode', self._handle_set_mode
        )
        self.srv_get_status = self.create_service(
            GetStatus, '/detection/get_status', self._handle_get_status
        )
        self.srv_compute = self.create_service(
            ComputeCoordinate, '/detection/compute_coordinate', self._handle_compute_coordinate
        )

        self._publish_mode()
        self.get_logger().info('detection_manager ready')

    def _publish_mode(self):
        m = UInt8()
        m.data = int(self.mode)
        self.pub_mode.publish(m)

    def _on_detection(self, msg: Detection):
        with self.mode_lock:
            mode = int(self.mode)

        try:
            if mode == MODE_OFF:
                return
            if mode == MODE_INITIALIZING and not self.initialized:
                return

            if mode == MODE_SENSOR_CRATE:
                det_type = getattr(msg, 'type', '').lower()
                text = getattr(msg, 'name', '')
                if not text:
                    return
                s = String()
                s.data = text
                if det_type == 'qr':
                    self.pub_qr.publish(s)
                elif det_type == 'hazmat':
                    self.pub_hazmat.publish(s)
                elif det_type == 'apriltag':
                    self.pub_apriltag.publish(s)
                return

            if mode == MODE_MAPPING:
                required = ['xmin', 'ymin', 'width', 'height', 'depth_center', 'camera_frame']
                for f in required:
                    if not hasattr(msg, f):
                        self.get_logger().warn(f'detection missing required field: {f}; dropping')
                        return

                success, point, message = compute_coordinate(
                    msg,
                    camera_info=None,
                    target_frame=self.target_frame,
                    tf_buffer=self.tf_buffer,
                    tf_timeout_sec=self.tf_timeout_sec,
                    fallback_intrinsics=self.fallback_intrinsics,
                )

                # --- THE FIX ---
                # If coordinate computation fails, log the error but still write the
                # available data to the CSV with empty coordinates.
                x_target, y_target, z_target = ('', '', '')
                if success:
                    x_target, y_target, z_target = (point.point.x, point.point.y, point.point.z)
                else:
                    self.get_logger().warn(
                        f'compute_coordinate failed: {message}. Writing to CSV with null coordinates.'
                    )
                    self.last_error = message
                # --- END OF FIX ---

                row = {
                    'timestamp_iso': f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec}',
                    'detection_id': getattr(msg, 'detection', -1),
                    'type': getattr(msg, 'type', ''),
                    'name': getattr(msg, 'name', ''),
                    'confidence': getattr(msg, 'confidence', 0.0),
                    'robot': getattr(msg, 'robot', self.robot),
                    'detection_mode': getattr(msg, 'mode', ''),
                    'camera_frame': getattr(msg, 'camera_frame', ''),
                    'target_frame': self.target_frame,
                    'x_target': x_target,
                    'y_target': y_target,
                    'z_target': z_target,
                    'u_center': getattr(msg, 'xmin', 0) + getattr(msg, 'width', 0) / 2.0,
                    'v_center': getattr(msg, 'ymin', 0) + getattr(msg, 'height', 0) / 2.0,
                    'depth_m': getattr(msg, 'depth_center', 0.0),
                    'bbox_xmin': getattr(msg, 'xmin', 0),
                    'bbox_ymin': getattr(msg, 'ymin', 0),
                    'bbox_w': getattr(msg, 'width', 0),
                    'bbox_h': getattr(msg, 'height', 0),
                }

                if self.csv_writer:
                    if not self.csv_writer.append_row(row):
                        self.last_error = 'csv write failed'
                        self.get_logger().error('CSV append_row returned False')
                else:
                    self.get_logger().warn('CSV writer not initialized in MAPPING mode')
                return

        except Exception as e:
            self.get_logger().error(f'error processing detection: {e}\n{traceback.format_exc()}')

    def _handle_set_mode(self, request, response):
        with self.mode_lock:
            prev = int(self.mode)
            req_mode = int(request.mode)
            if req_mode not in (MODE_OFF, MODE_INITIALIZING, MODE_SENSOR_CRATE, MODE_MAPPING):
                response.success = False
                response.previous_mode = prev
                response.message = f'invalid mode {req_mode}'
                return response

            if prev == MODE_MAPPING and req_mode != MODE_MAPPING and self.csv_writer:
                self.csv_writer.close()
                self.csv_writer = None

            if req_mode == MODE_INITIALIZING:
                self.initialized = False
                try:
                    self.tf_buffer.lookup_transform(
                        self.target_frame,
                        self.target_frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec),
                    )
                    csv_path = request.csv_path or self.csv_default_path
                    tmp_writer = CSVWriter(
                        csv_path,
                        DEFAULT_CSV_HEADER,
                        fsync=self.csv_fsync,
                        background=self.csv_background,
                    )
                    tmp_writer.close()
                    self.initialized = True
                    self.csv_path = csv_path
                    response.message = 'initialized checks passed'
                except Exception as e:
                    self.initialized = False
                    self.last_error = str(e)
                    response.message = f'initializing failed: {e}'

            if req_mode == MODE_MAPPING:
                csv_path = (
                    request.csv_path or self.csv_path
                )  # Use specified path or the one from init
                try:
                    self.csv_writer = CSVWriter(
                        csv_path,
                        DEFAULT_CSV_HEADER,
                        fsync=self.csv_fsync,
                        background=self.csv_background,
                    )
                    self.csv_path = csv_path
                except Exception as e:
                    response.success = False
                    response.previous_mode = prev
                    response.message = f'csv open failed: {e}'
                    return response

            self.mode = req_mode
            response.success = True
            response.previous_mode = prev
            response.message = 'mode set'
            self._publish_mode()
            return response

    def _handle_get_status(self, request, response):
        response.current_mode = int(self.mode)
        response.initialized = bool(self.initialized)
        response.csv_path = str(self.csv_path)
        response.last_error = str(self.last_error)
        return response

    def _handle_compute_coordinate(self, request, response):
        with self.mode_lock:
            mode = int(self.mode)
        if mode == MODE_OFF:
            response.success = False
            response.message = 'service not available in OFF mode'
            return response

        success, point, message = compute_coordinate(
            request.detection,
            request.camera_info,
            request.target_frame or self.target_frame,
            self.tf_buffer,
            self.tf_timeout_sec,
            self.fallback_intrinsics,
        )
        response.success = bool(success)
        if success and point:
            response.point = point
            response.message = message or ''
        else:
            response.point = PointStamped()
            response.message = message or 'compute failed'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DetectionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('shutting down detection_manager')
        if node.csv_writer:
            node.csv_writer.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
