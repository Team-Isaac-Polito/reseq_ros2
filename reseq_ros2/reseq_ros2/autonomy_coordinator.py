from __future__ import annotations

import math
import traceback

import rclpy
from geometry_msgs.msg import Point32, Polygon as PolygonMsg, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from reseq_interfaces.srv import SetMode

class AutonomyCoordinator(Node):
    def __init__(self):
        super().__init__('autonomy_coordinator')

        self.enabled_topic = self.declare_parameter('enabled_topic', '/autonomy/enabled').value
        self.status_topic = self.declare_parameter(
            'status_topic', '/autonomy/explorer_status'
        ).value
        self.planned_path_topic = self.declare_parameter(
            'planned_path_topic', '/autonomy/planned_path'
        ).value
        self.user_path_topic = self.declare_parameter(
            'user_path_topic', '/autonomy/user_path'
        ).value
        self.detection_service = self.declare_parameter(
            'detection_service', '/detection/set_mode'
        ).value
        self.target_frame = self.declare_parameter('target_frame', 'map').value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').value
        self.loop_period = self.declare_parameter('loop_period', 2.0).value
        self.goal_tolerance = self.declare_parameter('goal_tolerance', 0.15).value
        self.goal_timeout = self.declare_parameter('goal_timeout', 90.0).value

        self.autonomy_enabled = False
        self.goal_handle = None
        self.active_goal = None
        self.goal_sent_time = None
        self.user_path_waypoints: list[tuple[float, float]] = []
        self.user_path_index = 0

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.path_pub = self.create_publisher(Path, self.planned_path_topic, 10)
        self.autonomy_enable_pub = self.create_publisher(Bool, '/autonomy/enabled', 10)
        self.nav_cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.mode_client = self.create_client(SetMode, self.detection_service)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._footprint_pub = self.create_publisher(PolygonMsg, '/robot_footprint_local', 10)

        self.create_subscription(Bool, self.enabled_topic, self.enabled_callback, 10)
        self.create_subscription(Path, self.user_path_topic, self.user_path_callback, 10)
        self.create_timer(self.loop_period, self.control_loop)
        self.create_timer(0.1, self._publish_footprint)  # 10 Hz footprint updates

    def enabled_callback(self, msg: Bool) -> None:
        if msg.data and not self.autonomy_enabled:
            self.enable_mapping_mode()
        if not msg.data:
            if self.goal_handle is not None:
                cancel_future = self.goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda _: self.publish_status('cancelled'))
                self.goal_handle = None
                self.active_goal = None
            self.user_path_index = 0
        self.autonomy_enabled = msg.data

    def user_path_callback(self, msg: Path) -> None:
        if not msg.poses:
            self.user_path_waypoints = []
            self.user_path_index = 0
            self.publish_path([])
            self.publish_status('user_path_cleared')
            return

        self.user_path_waypoints = [
            (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses
        ]
        self.user_path_index = 0
        self.publish_path(self.user_path_waypoints)
        self.publish_status(f'user_path_received:{len(self.user_path_waypoints)}')

        if self.goal_handle is not None:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda _: self.publish_status('goal_replaced'))
            self.goal_handle = None
            self.active_goal = None

    def enable_mapping_mode(self) -> None:
        if not self.mode_client.wait_for_service(timeout_sec=0.2):
            return
        request = SetMode.Request(mode=3, csv_path='')
        self.mode_client.call_async(request)

    def publish_status(self, status: str) -> None:
        self.status_pub.publish(String(data=status))

    def _publish_footprint(self) -> None:
        """Publish a parallelogram footprint covering chassis1 (head) to chassis4 (tail).

        The polygon is in base_link frame and is consumed by the local costmap
        via the footprint_topic parameter.  At 10 Hz this keeps the footprint
        accurate as the snake bends through turns.
        """
        hw = 0.12   # half-width of robot body (m)
        hl = 0.15   # extension beyond chassis1/chassis4 centre (m)

        try:
            t4 = self.tf_buffer.lookup_transform(
                'base_link', 'chassis4_link', rclpy.time.Time()
            )
            c4x = t4.transform.translation.x
            c4y = t4.transform.translation.y
        except TransformException:
            # Fallback: straight configuration
            c4x, c4y = -1.26, 0.0

        dist = math.sqrt(c4x ** 2 + c4y ** 2)
        if dist < 0.01:
            fx, fy = 1.0, 0.0
        else:
            # Unit vector from chassis4 toward chassis1 (= forward direction)
            fx, fy = -c4x / dist, -c4y / dist

        # Perpendicular (left side)
        px, py = -fy, fx

        # 4 corners of the parallelogram (counter-clockwise)
        corners = [
            (hl * fx + hw * px,             hl * fy + hw * py),              # chassis1 front-left
            (c4x - hl * fx + hw * px,       c4y - hl * fy + hw * py),        # chassis4 rear-left
            (c4x - hl * fx - hw * px,       c4y - hl * fy - hw * py),        # chassis4 rear-right
            (hl * fx - hw * px,             hl * fy - hw * py),              # chassis1 front-right
        ]

        msg = PolygonMsg()
        for x, y in corners:
            pt = Point32()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.0
            msg.points.append(pt)
        self._footprint_pub.publish(msg)

    def current_robot_pose(self) -> tuple[float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, self.robot_frame, rclpy.time.Time()
            )
            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
            )
        except TransformException:
            return None

    def publish_path(self, waypoints: list[tuple[float, float]]) -> None:
        path = Path()
        path.header.frame_id = self.target_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x_pos, y_pos in waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x_pos
            pose.pose.position.y = y_pos
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

    def select_user_path_waypoint(self) -> tuple[float, float] | None:
        if not self.user_path_waypoints:
            return None

        # Skip waypoints that the robot has already passed (within goal tolerance)
        pose = self.current_robot_pose()

        # Skip waypoints already within tolerance.
        if pose is not None:
            while (
                self.user_path_index < len(self.user_path_waypoints)
                and math.dist(self.user_path_waypoints[self.user_path_index], pose)
                < self.goal_tolerance
            ):
                self.user_path_index += 1

        if self.user_path_index >= len(self.user_path_waypoints):
            return None

        return self.user_path_waypoints[self.user_path_index]

    def send_goal(self, goal_xy: tuple[float, float]) -> None:
        """Send a Nav2 NavigateToPose goal to the given waypoint."""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.publish_status('nav2_unavailable')
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.target_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_xy[0]
        goal.pose.pose.position.y = goal_xy[1]

        # Compute heading from current position toward goal
        robot_pos = self.current_robot_pose()
        if robot_pos is not None:
            dx = goal_xy[0] - robot_pos[0]
            dy = goal_xy[1] - robot_pos[1]
            yaw = math.atan2(dy, dx)
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            goal.pose.pose.orientation.w = 1.0

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)
        self.active_goal = goal_xy
        self.goal_sent_time = self.get_clock().now()
        self.publish_status(f'goal_sent:{goal_xy[0]:.2f},{goal_xy[1]:.2f}')

    def goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.goal_handle = None
            self.active_goal = None
            self.publish_status('goal_rejected')
            return
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future) -> None:
        self.goal_handle = None
        self.active_goal = None
        result = future.result()

        if result.status == 4:
            # Force robot stop
            self.nav_cmd_pub.publish(Twist())
            if self.user_path_index < len(self.user_path_waypoints):
                self.user_path_index += 1
            self.publish_status('goal_succeeded')
        else:
            self.publish_status(f'goal_finished_status:{result.status}')

    def goal_timed_out(self) -> bool:
        if self.goal_sent_time is None:
            return False
        age = (self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9
        return age > self.goal_timeout

    def control_loop(self) -> None:
        if not self.autonomy_enabled:
            return

        if self.goal_handle is not None:
            if self.goal_timed_out():
                cancel_future = self.goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda _: self.publish_status('goal_timeout'))
                self.goal_handle = None
                self.active_goal = None
            return

        goal = self.select_user_path_waypoint()

        if goal is None:
            if not self.user_path_waypoints:
                self.publish_status('waiting_user_path')
            return
        self.send_goal(goal)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AutonomyCoordinator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.logging.get_logger('autonomy_coordinator').warn(
            'AutonomyCoordinator interrupted by user'
        )
    except Exception as err:
        rclpy.logging.get_logger('autonomy_coordinator').fatal(
            f'Error in the autonomy_coordinator node: {str(err)}\n{traceback.format_exc()}'
        )
    else:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
