"""ROS observations, command topics and local API transport."""

import base64
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor, TimeoutError
import io
import json
import math
from pathlib import Path
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from PIL import Image as PILImage
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Point32, Polygon, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CompressedImage, Image, JointState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .core import Grid, Tasks, clamp, convex_hull, transform_xy, yaw_of
from .models import LightNavSession

# ROS encoding: (PIL raw decoder, PIL image mode, bytes per pixel)
CAMERA_ENCODINGS = {
    'rgb8': ('RGB', 'RGB', 3),
    'bgr8': ('BGR', 'RGB', 3),
    'rgba8': ('RGBA', 'RGBA', 4),
    'bgra8': ('BGRA', 'RGBA', 4),
    'mono8': ('L', 'L', 1),
}


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


class RosInterface(Node):
    def __init__(self):
        super().__init__('traction_manager')
        defaults = {
            'api_port': 8765,
            'camera_topic': '/realsense/color/image_raw',
            'odom_topic': '/odometry/filtered',
            'map_topic': '/map',
            'candidate_topic': '/traction/nav2/candidate',
            'camera_max_age_s': 2.0,
            'sensor_max_age_s': 2.0,
            'camera_grace_s': 3.0,
            'zone_qr_timeout_s': 3.0,
            'prediction_max_age_s': 6.0,
            'model_timeout_s': 5.0,
            'model_period_s': 1.0,
            # LightNav-0 is trained on frames 0.25 s apart (video_fps 4 in its eval_config).
            'frame_period_s': 0.25,
            # LightNav sometimes predicts stop for a single frame; require a run of them.
            'stop_confirmations': 3,
            # Visual tasks: replace a subgoal with a newer prediction after this long, and
            # back up this far after this many predictions the robot cannot follow.
            'subgoal_refresh_s': 3.0,
            'escape_after_unreachable': 2,
            'backup_distance_m': 0.3,
            'backup_speed': 0.15,
            'lightnav_url': '',
            'foresight_service': '/legged_deployment/foresight_planner',
            'nav2_lifecycle_manager': '/lifecycle_manager_traction_navigation',
            'max_subgoal_distance_m': 2.0,
            'min_subgoal_distance_m': 0.12,
            'unknown_gap_m': 0.15,
            'max_linear_speed': 0.19,
            'max_angular_speed': 2.4912,
            # Nav2 uses ROS (+angular.z turns left); the traction stack turns right for
            # +angular.z on the robot and in Gazebo, so the gate flips the sign.
            'traction_turn_sign': -1.0,
            'num_modules': 3,
            'module_half_length': 0.23,
            'module_half_width': 0.16,
            # The arm camera sits 5 cm behind the front edge and its near clip is 0.28 m;
            # padding the footprint ahead of it keeps walls far enough away to be seen.
            'camera_standoff_m': 0.25,
            'record_directory': str(Path.home() / '.local/state/reseq/traction'),
        }
        self.cfg = {
            key: self.declare_parameter(key, value).value for key, value in defaults.items()
        }
        self.tasks = Tasks()
        self.requests = queue.Queue(maxsize=32)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.enable_client = self.create_client(SetBool, '/autonomy/enable')
        # The action server exists even while Nav2 is inactive, so ask its lifecycle manager.
        self.nav2_active_client = self.create_client(
            Trigger, self.cfg['nav2_lifecycle_manager'] + '/is_active'
        )
        self.nav2_active = False
        self.nav2_active_future = None
        self.nav_goal = None
        self.nav_pending = False
        self.nav_epoch = 0
        self.model_future = None
        self.model_context = None
        self.next_prediction = 0.0
        self.next_frame = 0.0
        self.last_prediction = None
        self.stop_claims = 0
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='traction-model')
        self.lightnav = LightNavSession(self.cfg['lightnav_url'], self.cfg['model_timeout_s'])
        self.foresight = None
        self.foresight_type = None
        self.received = {}
        self.images = deque(maxlen=20)
        self.latest_odom = None
        self.joints = {}
        self.grid = None
        self.footprint = []
        self.source = ''
        self.health = ''
        self.autonomy_enabled = False
        self.enabled_generation = None
        self.last_candidate = Twist()
        self.candidate_time = 0.0
        self.log_file = None
        self.recorded_images = 0
        self.last_camera_publish = 0.0
        self.last_state_log = 0.0
        self.gate_reason = 'idle'
        self.previous_gate_reason = 'idle'

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.output = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.status_pub = self.create_publisher(String, '/traction/status', 10)
        # Nav2 costmaps read <costmap>/footprint; nav2_params.yaml's footprint_topic is unused.
        self.footprint_pubs = [
            self.create_publisher(Polygon, '/local_costmap/footprint', 10),
            self.create_publisher(Polygon, '/global_costmap/footprint', 10),
        ]
        self.camera_pub = self.create_publisher(
            CompressedImage, '/traction/camera/image/compressed', 2
        )
        self.create_subscription(Image, self.cfg['camera_topic'], self._image_cb, sensor_qos)
        self.create_subscription(Odometry, self.cfg['odom_topic'], self._odom_cb, sensor_qos)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, sensor_qos)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 5)
        self.create_subscription(OccupancyGrid, self.cfg['map_topic'], self._map_cb, map_qos)
        self.create_subscription(Twist, self.cfg['candidate_topic'], self._candidate_cb, 5)
        self.create_subscription(String, '/autonomy/cmd_source', self._source_cb, 5)
        self.create_subscription(String, '/autonomy/health', self._health_cb, 5)
        self.create_subscription(Bool, '/autonomy/enabled', self._enabled_cb, 5)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self._tick, clock=self.steady)
        self.create_timer(0.2, self._publish_footprint, clock=self.steady)
        self.create_timer(2.0, self._check_nav2_active, clock=self.steady)
        self.server = self._make_server()
        self.http_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(f'Traction API: http://127.0.0.1:{self.cfg["api_port"]}')

    def _make_server(self):
        manager = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_POST(self):
                if (
                    self.headers.get('Origin')
                    or self.headers.get('Content-Type', '').split(';')[0] != 'application/json'
                ):
                    self._reply(403, {'error': 'Local JSON clients only'})
                    return
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 < length <= 8192:
                        raise ValueError('Request body must be between 1 and 8192 bytes')
                    arguments = json.loads(self.rfile.read(length))
                    if not isinstance(arguments, dict):
                        raise ValueError('Arguments must be an object')
                    if not self.path.startswith('/tools/'):
                        raise ValueError('Unknown endpoint')
                    future = Future()
                    try:
                        manager.requests.put_nowait(
                            (self.path[len('/tools/') :], arguments, future)
                        )
                    except queue.Full:
                        raise ValueError('Manager request queue is full')
                    try:
                        result = future.result(timeout=8)
                    except TimeoutError:
                        future.cancel()
                        raise ValueError('ROS executor did not handle the request in time')
                    self._reply(200, result)
                except Exception as exc:
                    self._reply(400, {'error': str(exc)})

            def _reply(self, code, payload):
                body = json.dumps(payload, allow_nan=False).encode()
                try:
                    self.send_response(code)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        return ThreadingHTTPServer(('127.0.0.1', int(self.cfg['api_port'])), Handler)

    def _pose(self, frame='map', stamp=None, source='base_link'):
        transform = self.tf.lookup_transform(
            frame, source, stamp or Time(), timeout=Duration(seconds=0)
        )
        t = transform.transform
        return (t.translation.x, t.translation.y, yaw_of(t.rotation))

    def _pose_json(self, frame='map', stamp=None, source='base_link'):
        x, y, yaw = self._pose(frame, stamp, source)
        return {'x': x, 'y': y, 'yaw': yaw}

    def _age(self, name):
        value = self.received.get(name)
        if value is None:
            return None
        return time.monotonic() - value

    def _health_cb(self, message):
        self.health = message.data

    def _odom_cb(self, message):
        self.latest_odom = message
        self.received['odom'] = time.monotonic()

    def _scan_cb(self, message):
        self.received['scan'] = time.monotonic()

    def _joint_cb(self, message):
        self.joints = {
            n: float(v) for n, v in zip(message.name, message.position) if math.isfinite(v)
        }

    def _map_cb(self, message):
        info = message.info
        if info.resolution <= 0 or len(message.data) != info.width * info.height:
            return
        origin = info.origin
        self.grid = Grid(
            info.width,
            info.height,
            info.resolution,
            (origin.position.x, origin.position.y, yaw_of(origin.orientation)),
            message.data,
            self.cfg['unknown_gap_m'],
        )
        self.map_frame = message.header.frame_id
        self.received['map'] = time.monotonic()

    def _candidate_cb(self, message):
        linear = message.linear.x
        angular = message.angular.z
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.last_candidate = Twist()
            self.candidate_time = 0.0
            return
        output = Twist()
        output.linear.x = clamp(linear, self.cfg['max_linear_speed'])
        turn = self.cfg['traction_turn_sign'] * angular
        output.angular.z = clamp(turn, self.cfg['max_angular_speed'])
        self.last_candidate = output
        self.candidate_time = time.monotonic()

    def _source_cb(self, message):
        self.source = message.data
        if self.source == 'teleop_override' and self.tasks.active:
            self._finish('cancelled', 'manual_override')

    def _enabled_cb(self, message):
        was_enabled = self.autonomy_enabled
        self.autonomy_enabled = message.data
        # A disable sent when the previous task ended may arrive after the next task starts.
        confirmed = self.enabled_generation == self.tasks.generation
        if was_enabled and not message.data and self.tasks.active and confirmed:
            self._finish('cancelled', 'autonomy_disabled')

    def _jpeg(self, message):
        if message.encoding not in CAMERA_ENCODINGS:
            raise ValueError(f'Unsupported camera encoding: {message.encoding}')
        decoder, image_mode, channels = CAMERA_ENCODINGS[message.encoding]
        width, height = message.width, message.height
        if width <= 0 or height <= 0 or width * height > 4096 * 4096:
            raise ValueError('Invalid image dimensions')
        if message.step < width * channels or len(message.data) < message.step * height:
            raise ValueError('Truncated camera image')
        image = PILImage.frombytes(
            image_mode, (width, height), bytes(message.data), 'raw', decoder, message.step, 1
        )
        image = image.convert('RGB')
        image.thumbnail((640, 480))
        result = io.BytesIO()
        image.save(result, format='JPEG', quality=85)
        return result.getvalue(), image.size

    def _image_cb(self, message):
        received = time.monotonic()
        self.images.append((message, received))
        self.received['camera'] = received
        if self.camera_pub.get_subscription_count() == 0:
            return
        if time.monotonic() - self.last_camera_publish < 0.2:
            return
        try:
            jpeg, _ = self._jpeg(message)
            compressed = CompressedImage()
            compressed.header = message.header
            compressed.format = 'jpeg'
            compressed.data = jpeg
            self.camera_pub.publish(compressed)
            self.last_camera_publish = time.monotonic()
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5)

    def _recent_image(self):
        """Return the newest image that is fresh and has odom transforms at its stamp."""
        max_age = self.cfg['camera_max_age_s']
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds / 1e9
        for message, received in reversed(self.images):
            # Wall-clock age still grows while Gazebo is paused.
            age = now - received
            if age > max_age:
                raise ValueError('No recent camera image')
            ros_age = ros_now - stamp_seconds(message.header.stamp)
            if ros_age < -0.1 or ros_age > max_age:
                continue
            stamp = Time.from_msg(message.header.stamp)
            try:
                pose = self._pose_json('odom', stamp)
                camera_pose = self._pose_json('odom', stamp, message.header.frame_id)
            except TransformException:
                continue
            return message, age, pose, camera_pose
        raise ValueError('No recent camera image with matching transforms')

    def _observation(self):
        message, age, pose, camera_pose = self._recent_image()
        jpeg, (width, height) = self._jpeg(message)
        return {
            'stamp_s': stamp_seconds(message.header.stamp),
            'camera_frame': message.header.frame_id,
            'pose_frame': 'odom',
            'robot_pose': pose,
            'camera_pose': camera_pose,
            'age_s': age,
            'width': width,
            'height': height,
            'jpeg': jpeg,
        }

    def _publish_footprint(self):
        a = self.cfg['module_half_length']
        b = self.cfg['module_half_width']
        points = []
        try:
            for i in range(1, int(self.cfg['num_modules']) + 1):
                pose = self._pose('base_link', source=f'chassis{i}_link')
                front = a + self.cfg['camera_standoff_m'] if i == 1 else a
                for corner in ((-a, -b), (front, -b), (front, b), (-a, b)):
                    points.append(transform_xy(corner, pose))
        except TransformException:
            self.footprint = []
            return
        self.footprint = convex_hull(points)
        polygon = Polygon()
        polygon.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in self.footprint]
        for publisher in self.footprint_pubs:
            publisher.publish(polygon)

    def _foresight_client(self):
        if self.foresight is None:
            from rosidl_runtime_py.utilities import get_service

            self.foresight_type = get_service('amrl_msgs/srv/ForesightPlannerSrv')
            self.foresight = self.create_client(self.foresight_type, self.cfg['foresight_service'])
        return self.foresight

    def _check_nav2_active(self):
        if self.nav2_active_future is not None and not self.nav2_active_future.done():
            return
        if not self.nav2_active_client.service_is_ready():
            self.nav2_active = False
            return
        self.nav2_active_future = self.nav2_active_client.call_async(Trigger.Request())
        self.nav2_active_future.add_done_callback(self._nav2_active_cb)

    def _nav2_active_cb(self, future):
        try:
            self.nav2_active = future.result().success
        except Exception:
            self.nav2_active = False

    def _validate_goal(self, x, y, yaw, frame):
        if self.grid is None or not self.footprint:
            raise ValueError('Map and robot footprint must be ready before setting a goal')
        if frame not in ('map', 'odom'):
            raise ValueError('frame must be map or odom')
        if frame != self.map_frame:
            frame_pose = self._pose(self.map_frame, source=frame)
            x, y = transform_xy((x, y), frame_pose)
            yaw += frame_pose[2]
        if not self.grid.pose_free((x, y, yaw), self.footprint):
            raise ValueError('Goal footprint intersects occupied, unknown or unmapped cells')
        return x, y, yaw

    def _readiness(self, visual=False):
        problems = []
        for name in ('odom', 'scan'):
            age = self._age(name)
            if age is None or age > self.cfg['sensor_max_age_s']:
                problems.append(f'stale_{name}')
        if self.grid is None:
            problems.append('no_map')
        if not self.footprint:
            problems.append('no_footprint_tf')
        if not self.nav.server_is_ready():
            problems.append('nav2_unavailable')
        elif not self.nav2_active:
            problems.append('nav2_inactive')
        try:
            self._pose('map')
        except TransformException:
            problems.append('no_map_pose')
        if visual:
            try:
                self._recent_image()
            except (ValueError, TransformException):
                problems.append('camera_not_ready')
        return problems

    def tool_get_state(self):
        try:
            pose = self._pose_json('map')
        except TransformException:
            pose = None
        ages = {name: self._age(name) for name in ('camera', 'odom', 'scan', 'map')}
        velocity = None
        if self.latest_odom is not None:
            velocity = {
                'linear_x': self.latest_odom.twist.twist.linear.x,
                'angular_z': self.latest_odom.twist.twist.angular.z,
            }
        active_task = None
        if self.tasks.active is not None:
            active_task = self.tasks.active.view(time.monotonic())
        problems = self._readiness()
        return {
            'pose': pose,
            'pose_frame': 'map',
            'observation_ages_s': ages,
            'velocity': velocity,
            'joints': self.joints,
            'command_source': self.source,
            'mux_health': self.health,
            'gate': self.gate_reason,
            'ready': not problems,
            'readiness_problems': problems,
            'autonomy_enabled': self.autonomy_enabled,
            'active_task': active_task,
            'model_config': {
                'lightnav_url_configured': bool(self.cfg['lightnav_url']),
                'foresight_service': self.cfg['foresight_service'],
            },
        }

    def tool_observe(self):
        observation = self._observation()
        observation['jpeg_base64'] = base64.b64encode(observation.pop('jpeg')).decode()
        return observation
