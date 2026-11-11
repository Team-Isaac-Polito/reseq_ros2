"""ROS side of the traction manager: sensors, command gate input, footprint and state."""

import base64
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import math
import queue
import threading
import time

from geometry_msgs.msg import Point32, Polygon, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image, JointState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .api_server import make_server
from .camera import to_jpeg
from .core import Grid, Tasks, clamp, convex_hull, transform_xy, yaw_of
from .models import LightNavSession
from .parameters import DEFAULTS


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


class RosInterface(Node):
    def __init__(self):
        super().__init__('traction_manager')
        self.cfg = {}
        for name, default in DEFAULTS.items():
            self.cfg[name] = self.declare_parameter(name, default).value

        self.tasks = Tasks()
        self.requests = queue.Queue(maxsize=32)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        # Nav2
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.enable_client = self.create_client(SetBool, '/autonomy/enable')
        # The action server exists even while Nav2 is inactive, so ask its lifecycle manager.
        active_service = self.cfg['nav2_lifecycle_manager'] + '/is_active'
        self.nav2_active_client = self.create_client(Trigger, active_service)
        self.nav2_active = False
        self.nav2_active_future = None
        self.nav_goal = None
        self.nav_pending = False
        self.nav_epoch = 0

        # Models
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

        # Latest sensor data and robot state
        self.received = {}  # topic name -> time.monotonic() of the last message
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

        # Task records and status
        self.log_file = None
        self.recorded_images = 0
        self.last_camera_publish = 0.0
        self.last_state_log = 0.0
        self.gate_reason = 'idle'
        self.previous_gate_reason = 'idle'

        self._create_topics()
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self._tick, clock=self.steady)
        self.create_timer(0.2, self._publish_footprint, clock=self.steady)
        self.create_timer(2.0, self._check_nav2_active, clock=self.steady)

        self.server = make_server(self, int(self.cfg['api_port']))
        self.http_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(f'Traction API: http://127.0.0.1:{self.cfg["api_port"]}')

    def _create_topics(self):
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        # The map is published once and kept for late subscribers.
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cfg = self.cfg

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

        self.create_subscription(Image, cfg['camera_topic'], self._image_cb, sensor_qos)
        self.create_subscription(Odometry, cfg['odom_topic'], self._odom_cb, sensor_qos)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, sensor_qos)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 5)
        self.create_subscription(OccupancyGrid, cfg['map_topic'], self._map_cb, map_qos)
        self.create_subscription(Twist, cfg['candidate_topic'], self._candidate_cb, 5)
        self.create_subscription(String, '/autonomy/cmd_source', self._source_cb, 5)
        self.create_subscription(String, '/autonomy/health', self._health_cb, 5)
        self.create_subscription(Bool, '/autonomy/enabled', self._enabled_cb, 5)

    # ---- Poses ----

    def _pose(self, frame='map', stamp=None, source='base_link'):
        """Return the (x, y, yaw) of source in frame. stamp None means the latest."""
        if stamp is None:
            stamp = Time()
        transform = self.tf.lookup_transform(frame, source, stamp, timeout=Duration(seconds=0))
        t = transform.transform
        return (t.translation.x, t.translation.y, yaw_of(t.rotation))

    def _pose_json(self, frame='map', stamp=None, source='base_link'):
        x, y, yaw = self._pose(frame, stamp, source)
        return {'x': x, 'y': y, 'yaw': yaw}

    def _age(self, name):
        """Seconds since the last message of this kind, or None if none arrived."""
        value = self.received.get(name)
        if value is None:
            return None
        return time.monotonic() - value

    # ---- Topic callbacks ----

    def _health_cb(self, message):
        self.health = message.data

    def _odom_cb(self, message):
        self.latest_odom = message
        self.received['odom'] = time.monotonic()

    def _scan_cb(self, message):
        self.received['scan'] = time.monotonic()

    def _joint_cb(self, message):
        joints = {}
        for name, position in zip(message.name, message.position):
            if math.isfinite(position):
                joints[name] = float(position)
        self.joints = joints

    def _map_cb(self, message):
        info = message.info
        if info.resolution <= 0:
            return
        if len(message.data) != info.width * info.height:
            return
        position = info.origin.position
        origin = (position.x, position.y, yaw_of(info.origin.orientation))
        self.grid = Grid(
            info.width,
            info.height,
            info.resolution,
            origin,
            message.data,
            self.cfg['unknown_gap_m'],
        )
        self.map_frame = message.header.frame_id
        self.received['map'] = time.monotonic()

    def _candidate_cb(self, message):
        """Nav2's command. Limit it and convert it to the traction stack's turn sign."""
        linear = message.linear.x
        angular = message.angular.z
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.last_candidate = Twist()
            self.candidate_time = 0.0
            return
        turn = self.cfg['traction_turn_sign'] * angular
        output = Twist()
        output.linear.x = clamp(linear, self.cfg['max_linear_speed'])
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
        if not was_enabled or message.data or not self.tasks.active:
            return
        # A disable sent when the previous task ended may arrive after the next task starts.
        if self.enabled_generation == self.tasks.generation:
            self._finish('cancelled', 'autonomy_disabled')

    # ---- Camera ----

    def _jpeg(self, message):
        return to_jpeg(message)

    def _image_cb(self, message):
        received = time.monotonic()
        self.images.append((message, received))
        self.received['camera'] = received
        # Republish a small JPEG at most 5 times a second, only if someone listens.
        if self.camera_pub.get_subscription_count() == 0:
            return
        if received - self.last_camera_publish < 0.2:
            return
        try:
            jpeg, _ = self._jpeg(message)
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5)
            return
        compressed = CompressedImage()
        compressed.header = message.header
        compressed.format = 'jpeg'
        compressed.data = jpeg
        self.camera_pub.publish(compressed)
        self.last_camera_publish = time.monotonic()

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
        """The newest usable image with the robot and camera poses when it was taken."""
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

    # ---- Footprint ----

    def _module_corners(self, index):
        """Corners of one module in its own frame. The first module is padded in front."""
        a = self.cfg['module_half_length']
        b = self.cfg['module_half_width']
        front = a
        if index == 1:
            front = a + self.cfg['camera_standoff_m']
        return [(-a, -b), (front, -b), (front, b), (-a, b)]

    def _publish_footprint(self):
        """Publish the hull around all modules, so Nav2 sees the robot's current shape."""
        points = []
        try:
            for index in range(1, int(self.cfg['num_modules']) + 1):
                pose = self._pose('base_link', source=f'chassis{index}_link')
                for corner in self._module_corners(index):
                    points.append(transform_xy(corner, pose))
        except TransformException:
            self.footprint = []
            return
        self.footprint = convex_hull(points)
        polygon = Polygon()
        for x, y in self.footprint:
            polygon.points.append(Point32(x=float(x), y=float(y), z=0.0))
        for publisher in self.footprint_pubs:
            publisher.publish(polygon)

    # ---- Services ----

    def _foresight_client(self):
        """Create the Foresight client on first use; its message package is optional."""
        if self.foresight is None:
            from rosidl_runtime_py.utilities import get_service

            self.foresight_type = get_service('amrl_msgs/srv/ForesightPlannerSrv')
            self.foresight = self.create_client(self.foresight_type, self.cfg['foresight_service'])
        return self.foresight

    def _check_nav2_active(self):
        waiting = self.nav2_active_future is not None and not self.nav2_active_future.done()
        if waiting:
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

    # ---- Checks ----

    def _validate_goal(self, x, y, yaw, frame):
        """Return the goal in the map frame, or raise if the robot cannot stand there."""
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
        """Return a list of problems that block a task. Empty means ready."""
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

    # ---- Tools ----

    def tool_get_state(self):
        try:
            pose = self._pose_json('map')
        except TransformException:
            pose = None
        ages = {}
        for name in ('camera', 'odom', 'scan', 'map'):
            ages[name] = self._age(name)
        velocity = None
        if self.latest_odom is not None:
            twist = self.latest_odom.twist.twist
            velocity = {'linear_x': twist.linear.x, 'angular_z': twist.angular.z}
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
        jpeg = observation.pop('jpeg')
        observation['jpeg_base64'] = base64.b64encode(jpeg).decode()
        return observation
