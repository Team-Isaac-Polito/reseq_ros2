# detection_manager_node.py
import os
import threading

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
        mode_qos = QoSProfile(depth=1)
        mode_qos.reliability = ReliabilityPolicy.RELIABLE
        mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        det_qos = QoSProfile(depth=5)
        det_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        det_qos.history = HistoryPolicy.KEEP_LAST

        text_qos = QoSProfile(depth=10)
        text_qos.reliability = ReliabilityPolicy.RELIABLE

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

        # publish initial mode (transient_local so late subscribers see it)
        self._publish_mode()

        self.get_logger().info('detection_manager ready')

    # mode publisher helper
    def _publish_mode(self):
        m = UInt8()
        m.data = int(self.mode)
        self.pub_mode.publish(m)

    # detection callback
    def _on_detection(self, msg: Detection):
        # quick local snapshot of mode
        with self.mode_lock:
            mode = int(self.mode)

        try:
            # drop in OFF
            if mode == MODE_OFF:
                return

            if mode == MODE_INITIALIZING and not self.initialized:
                return

            # SENSOR_CRATE: process only "others" pipeline
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

            # MAPPING: process all types
            if mode == MODE_MAPPING:
                # validate required fields
                required = ['xmin', 'ymin', 'width', 'height', 'depth_center', 'camera_frame']
                for f in required:
                    if not hasattr(msg, f):
                        self.get_logger().warn(f'detection missing required field: {f}; dropping')
                        return
                # perform coordinate computation
                success, point, message = compute_coordinate(
                    msg,
                    camera_info=None,
                    target_frame=self.target_frame,
                    tf_buffer=self.tf_buffer,
                    tf_timeout_sec=self.tf_timeout_sec,
                    fallback_intrinsics=self.fallback_intrinsics,
                )
                if not success:
                    self.get_logger().warn(f'compute_coordinate failed: {message}')
                    self.last_error = message
                    return
                # append CSV
                row = {
                    'timestamp_iso': getattr(msg, 'time', None)
                    and f'{msg.time.sec}.{msg.time.nanosec}'
                    or '',
                    'detection_id': getattr(msg, 'detection', -1),
                    'type': getattr(msg, 'type', ''),
                    'name': getattr(msg, 'name', ''),
                    'confidence': getattr(msg, 'confidence', 0.0),
                    'robot': getattr(msg, 'robot', self.robot),
                    'detection_mode': getattr(msg, 'mode', ''),
                    'camera_frame': getattr(msg, 'camera_frame', ''),
                    'target_frame': self.target_frame,
                    'x_target': point.point.x,
                    'y_target': point.point.y,
                    'z_target': point.point.z,
                    'u_center': getattr(msg, 'xmin', 0) + getattr(msg, 'width', 0) / 2.0,
                    'v_center': getattr(msg, 'ymin', 0) + getattr(msg, 'height', 0) / 2.0,
                    'depth_m': getattr(msg, 'depth_center', 0.0),
                    'bbox_xmin': getattr(msg, 'xmin', 0),
                    'bbox_ymin': getattr(msg, 'ymin', 0),
                    'bbox_w': getattr(msg, 'width', 0),
                    'bbox_h': getattr(msg, 'height', 0),
                }
                if self.csv_writer:
                    ok = self.csv_writer.append_row(row)
                    if not ok:
                        self.last_error = 'csv write failed'
                        self.get_logger().error('CSV append_row returned False')
                else:
                    self.get_logger().warn('CSV writer not initialized in MAPPING mode')
                return

        except Exception as e:
            self.get_logger().error(f'error processing detection: {e}')

    # service handlers
    def _handle_set_mode(self, request, response):
        with self.mode_lock:
            prev = int(self.mode)
            req_mode = int(request.mode)
            # validate
            if req_mode not in (MODE_OFF, MODE_INITIALIZING, MODE_SENSOR_CRATE, MODE_MAPPING):
                response.success = False
                response.previous_mode = prev
                response.message = f'invalid mode {req_mode}'
                return response

            # handle leaving MAPPING: close csv
            if prev == MODE_MAPPING and req_mode != MODE_MAPPING:
                if self.csv_writer:
                    self.csv_writer.close()
                    self.csv_writer = None

            # handle entering INITIALIZING
            if req_mode == MODE_INITIALIZING:
                self.initialized = False
                # attempt quick TF check and csv open test (do minimal prechecks)
                try:
                    # test tf by attempting a zero-time lookup from target_frame to target_frame
                    self.tf_buffer.lookup_transform(
                        self.target_frame,
                        self.target_frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec),
                    )
                    # csv open
                    csv_path = request.csv_path if request.csv_path else self.csv_default_path
                    # try to open file for append (will create dirs)
                    header = DEFAULT_CSV_HEADER
                    # open and close immediately to test permission
                    tmp_writer = CSVWriter(
                        csv_path, header, fsync=self.csv_fsync, background=self.csv_background
                    )
                    tmp_writer.close()
                    # all good
                    self.initialized = True
                    self.csv_path = csv_path
                    response.message = 'initialized checks passed'
                except Exception as e:
                    self.initialized = False
                    self.last_error = str(e)
                    response.message = f'initializing failed: {e}'

            # handle entering MAPPING
            if req_mode == MODE_MAPPING:
                # ensure initialized
                csv_path = request.csv_path if request.csv_path else self.csv_default_path
                # open CSV writer
                try:
                    header = DEFAULT_CSV_HEADER
                    self.csv_writer = CSVWriter(
                        csv_path, header, fsync=self.csv_fsync, background=self.csv_background
                    )
                    self.csv_path = csv_path
                except Exception as e:
                    response.success = False
                    response.previous_mode = prev
                    response.message = f'csv open failed: {e}'
                    return response

            # set new mode
            self.mode = req_mode
            response.success = True
            response.previous_mode = prev
            response.message = 'mode set'
            # publish new mode (transient_local so detector sees it)
            self._publish_mode()
            return response

    def _handle_get_status(self, request, response):
        response.current_mode = int(self.mode)
        response.initialized = bool(self.initialized)
        response.csv_path = str(self.csv_path)
        response.last_error = str(self.last_error)
        return response

    def _handle_compute_coordinate(self, request, response):
        # in OFF mode, return failure
        with self.mode_lock:
            mode = int(self.mode)
        if mode == MODE_OFF:
            response.success = False
            response.message = 'service not available in OFF mode'
            return response

        det = request.detection
        camera_info = (
            request.camera_info
            if hasattr(request, 'camera_info') and request.camera_info is not None
            else None
        )
        tgt = request.target_frame if request.target_frame else self.target_frame

        success, point, message = compute_coordinate(
            det, camera_info, tgt, self.tf_buffer, self.tf_timeout_sec, self.fallback_intrinsics
        )
        response.success = bool(success)
        if success and point is not None:
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

