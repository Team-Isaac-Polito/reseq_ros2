from __future__ import annotations

import math
import traceback
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from reseq_interfaces.srv import SetMode


def grid_index(width: int, x: int, y: int) -> int:
    return y * width + x


def frontier_centroids(
    data: list[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    min_cluster_size: int,
) -> list[tuple[float, float]]:
    def is_frontier(x: int, y: int) -> bool:
        idx = grid_index(width, x, y)
        if data[idx] != 0:
            return False
        for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
            if 0 <= nx < width and 0 <= ny < height:
                if data[grid_index(width, nx, ny)] == -1:
                    return True
        return False

    visited = set()
    centroids = []
    for y in range(height):
        for x in range(width):
            if (x, y) in visited or not is_frontier(x, y):
                continue
            queue = deque([(x, y)])
            cluster = []
            visited.add((x, y))
            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))
                for nx in range(cx - 1, cx + 2):
                    for ny in range(cy - 1, cy + 2):
                        if (nx, ny) in visited:
                            continue
                        if 0 <= nx < width and 0 <= ny < height and is_frontier(nx, ny):
                            visited.add((nx, ny))
                            queue.append((nx, ny))
            if len(cluster) < min_cluster_size:
                continue
            avg_x = sum(point[0] for point in cluster) / len(cluster)
            avg_y = sum(point[1] for point in cluster) / len(cluster)
            centroids.append((origin_x + avg_x * resolution, origin_y + avg_y * resolution))
    return centroids


def trajectory_waypoints_from_map(
    data: list[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    row_stride_cells: int,
    edge_margin_cells: int,
    min_row_width_cells: int,
) -> list[tuple[float, float]]:
    row_stride_cells = max(1, row_stride_cells)
    edge_margin_cells = max(0, edge_margin_cells)
    waypoints = []
    reverse = False

    for y in range(0, height, row_stride_cells):
        free_x = [x for x in range(width) if data[grid_index(width, x, y)] == 0]
        if len(free_x) < min_row_width_cells:
            continue

        left = min(free_x) + edge_margin_cells
        right = max(free_x) - edge_margin_cells
        if right - left + 1 < min_row_width_cells:
            continue

        row_y = origin_y + y * resolution
        left_xy = (origin_x + left * resolution, row_y)
        right_xy = (origin_x + right * resolution, row_y)
        if reverse:
            waypoints.extend([right_xy, left_xy])
        else:
            waypoints.extend([left_xy, right_xy])
        reverse = not reverse

    return waypoints


class AutonomyCoordinator(Node):
    def __init__(self):
        super().__init__('autonomy_coordinator')

        self.enabled_topic = self.declare_parameter('enabled_topic', '/autonomy/enabled').value
        self.planner_mode = self.declare_parameter('planner_mode', 'user_path').value
        self.map_topic = self.declare_parameter('map_topic', '/map').value
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
        self.frontier_min_cluster_size = self.declare_parameter(
            'frontier_min_cluster_size', 8
        ).value
        self.frontier_min_distance = self.declare_parameter('frontier_min_distance', 0.6).value
        self.frontier_goal_timeout = self.declare_parameter('frontier_goal_timeout', 90.0).value
        self.goal_progress_timeout = self.declare_parameter('goal_progress_timeout', 25.0).value
        self.trajectory_row_stride_cells = self.declare_parameter(
            'trajectory_row_stride_cells', 8
        ).value
        self.trajectory_edge_margin_cells = self.declare_parameter(
            'trajectory_edge_margin_cells', 2
        ).value
        self.trajectory_min_row_width_cells = self.declare_parameter(
            'trajectory_min_row_width_cells', 6
        ).value

        self.autonomy_enabled = False
        self.latest_map = None
        self.goal_handle = None
        self.active_goal = None
        self.goal_sent_time = None
        self.last_progress_time = None
        self.trajectory_waypoints = []
        self.trajectory_index = 0
        self.user_path_waypoints = []
        self.user_path_index = 0

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.path_pub = self.create_publisher(Path, self.planned_path_topic, 10)
        self.autonomy_enable_pub = self.create_publisher(Bool, '/autonomy/enabled', 10)
        self.nav_cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.mode_client = self.create_client(SetMode, self.detection_service)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Bool, self.enabled_topic, self.enabled_callback, 10)
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        self.create_subscription(Path, self.user_path_topic, self.user_path_callback, 10)
        self.create_timer(self.loop_period, self.control_loop)

    def enabled_callback(self, msg: Bool) -> None:
        if msg.data and not self.autonomy_enabled:
            self.enable_mapping_mode()
        if not msg.data and self.goal_handle is not None:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda _: self.publish_status('cancelled'))
            self.goal_handle = None
            self.active_goal = None
            self.trajectory_index = 0
            self.user_path_index = 0
        self.autonomy_enabled = msg.data

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

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

    def select_frontier(self) -> tuple[float, float] | None:
        if self.latest_map is None:
            return None

        map_msg = self.latest_map
        candidates = frontier_centroids(
            list(map_msg.data),
            map_msg.info.width,
            map_msg.info.height,
            map_msg.info.resolution,
            map_msg.info.origin.position.x,
            map_msg.info.origin.position.y,
            self.frontier_min_cluster_size,
        )
        pose = self.current_robot_pose()
        if pose is None or not candidates:
            return None

        filtered = []
        for candidate in candidates:
            distance = math.dist(candidate, pose)
            if distance >= self.frontier_min_distance:
                filtered.append((distance, candidate))

        if not filtered:
            return None
        filtered.sort(key=lambda item: item[0])
        return filtered[0][1]

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

    def generate_trajectory(self) -> list[tuple[float, float]]:
        if self.latest_map is None:
            return []

        map_msg = self.latest_map
        return trajectory_waypoints_from_map(
            list(map_msg.data),
            map_msg.info.width,
            map_msg.info.height,
            map_msg.info.resolution,
            map_msg.info.origin.position.x,
            map_msg.info.origin.position.y,
            self.trajectory_row_stride_cells,
            self.trajectory_edge_margin_cells,
            self.trajectory_min_row_width_cells,
        )

    def select_trajectory_waypoint(self) -> tuple[float, float] | None:
        if not self.trajectory_waypoints:
            self.trajectory_waypoints = self.generate_trajectory()
            self.trajectory_index = 0
            self.publish_path(self.trajectory_waypoints)

        pose = self.current_robot_pose()
        while pose is not None and self.trajectory_index < len(self.trajectory_waypoints):
            candidate = self.trajectory_waypoints[self.trajectory_index]
            if math.dist(candidate, pose) >= self.frontier_min_distance:
                return candidate
            self.trajectory_index += 1

        return None

    def select_user_path_waypoint(self) -> tuple[float, float] | None:
        if not self.user_path_waypoints:
            return None

        # Skip waypoints that the robot has already passed (within goal tolerance)
        pose = self.current_robot_pose()
        if pose is not None:
            while (
                self.user_path_index < len(self.user_path_waypoints)
                and math.dist(self.user_path_waypoints[self.user_path_index], pose) < 0.15
            ):
                self.user_path_index += 1

        if self.user_path_index >= len(self.user_path_waypoints):
            return None

        return self.user_path_waypoints[self.user_path_index]

    def send_goal(self, goal_xy: tuple[float, float]) -> None:
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
        self.last_progress_time = self.goal_sent_time
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

            if self.planner_mode == 'user_path' and self.user_path_index < len(
                self.user_path_waypoints
            ):
                self.user_path_index += 1
            if self.planner_mode == 'trajectory' and self.trajectory_index < len(
                self.trajectory_waypoints
            ):
                self.trajectory_index += 1
            self.publish_status('goal_succeeded')
        else:
            self.publish_status(f'goal_finished_status:{result.status}')

    def goal_timed_out(self) -> bool:
        if self.goal_sent_time is None:
            return False
        age = (self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9
        return age > self.frontier_goal_timeout

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

        if self.planner_mode == 'user_path':
            goal = self.select_user_path_waypoint()
        elif self.planner_mode == 'trajectory':
            goal = self.select_trajectory_waypoint()
            if goal is None:
                self.trajectory_waypoints = self.generate_trajectory()
                self.trajectory_index = 0
                self.publish_path(self.trajectory_waypoints)
                goal = self.select_trajectory_waypoint()
        else:
            goal = self.select_frontier()

        if goal is None:
            if self.planner_mode == 'user_path':
                self.publish_status('waiting_user_path')
                return
            self.publish_status('no_goal_available')
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
