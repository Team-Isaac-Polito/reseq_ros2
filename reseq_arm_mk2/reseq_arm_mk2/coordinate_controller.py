#!/usr/bin/env python3
"""Coordinate-based arm controller with Reachability Map support."""

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Quaternion
from moveit_msgs.srv import GetPositionIK
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Try importing reach and scipy for the map functionality
try:
    import reach
    from scipy.spatial import KDTree

    HAS_REACH_SUPPORT = True
except ImportError as e:
    HAS_REACH_SUPPORT = False
    IMPORT_ERROR_MSG = str(e)


class CoordinateController(Node):
    def __init__(self):
        super().__init__('coordinate_controller')

        # Parameters
        self.declare_parameter('reachability_db_path', 'src/reseq_arm_mk2/config/reach.db.xml')
        self.declare_parameter('reachability_threshold', 0.02)  # Meters

        self.ik_service = self.create_client(GetPositionIK, '/compute_ik')
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )

        self.current_joint_state = None
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Point, '~/move_to_point', self.move_to_point_callback, 10)

        self.joint_names = [
            'mod1__base_pitch_arm_joint',
            'mod1__base_roll_arm_joint',
            'mod1__elbow_pitch_arm_joint',
            'mod1__forearm_roll_arm_joint',
            'mod1__wrist_pitch_arm_joint',
            'mod1__wrist_roll_arm_joint',
        ]

        # Initialize Reachability Map
        self.kdtree = None
        self.load_reachability_map()

        self.get_logger().info('CoordinateController initialized. Waiting for /joint_states...')

    def load_reachability_map(self):
        """Loads the reachability database and builds a KDTree for nearest neighbor search."""
        db_path = self.get_parameter('reachability_db_path').get_parameter_value().string_value

        if not db_path:
            self.get_logger().info(
                "No 'reachability_db_path' parameter provided. Reachability check disabled."
            )
            return

        if not HAS_REACH_SUPPORT:
            self.get_logger().error(f'Cannot load reachability map: {IMPORT_ERROR_MSG}')
            return

        try:
            self.get_logger().info(f'Loading reachability database from: {db_path}')
            db = reach.load(db_path)

            # Robustly access db.results (Handle both property and method cases)
            if callable(db.results):
                results_list = db.results()
            else:
                results_list = db.results

            if not results_list:
                self.get_logger().warn('Reachability map loaded but contains 0 results.')
                return

            # Get the latest results
            results = results_list[-1]

            reachable_points = []
            for record in results:
                if record.reached:
                    # FIX: Call goal() as a method to get the numpy array
                    if callable(record.goal):
                        pose_matrix = record.goal()
                    else:
                        pose_matrix = record.goal

                    pos = pose_matrix[:3, 3]
                    reachable_points.append(pos)

            if not reachable_points:
                self.get_logger().warn('Reachability map loaded but contains 0 reachable points.')
                return

            self.kdtree = KDTree(reachable_points)
            self.get_logger().info(
                f'Reachability map successfully loaded with '
                f'{len(reachable_points)} reachable points.'
            )

        except Exception as e:
            self.get_logger().error(f'Failed to load reachability map: {e}')

    def joint_state_callback(self, msg: JointState):
        if self.current_joint_state is None:
            self.get_logger().info('First joint state received.')
        self.current_joint_state = msg

    def move_to_point_callback(self, msg: Point):
        self.get_logger().info(f'Received new target: X={msg.x}, Y={msg.y}, Z={msg.z}')
        self.move_to_coordinate(msg.x, msg.y, msg.z, duration_seconds=5.0)

    def move_to_coordinate(self, x: float, y: float, z: float, duration_seconds: float = 5.0):
        # Reachability Check and Snapping
        if self.kdtree is not None:
            target_arr = np.array([x, y, z])
            dist, idx = self.kdtree.query(target_arr)
            threshold = (
                self.get_parameter('reachability_threshold').get_parameter_value().double_value
            )

            if dist > threshold:
                closest_point = self.kdtree.data[idx]
                self.get_logger().warn(
                    f'Target unreachable (dist={dist:.3f}m). Snapping to: '
                    f'({closest_point[0]:.3f}, {closest_point[1]:.3f}, '
                    f'{closest_point[2]:.3f})'
                )
                x, y, z = (
                    float(closest_point[0]),
                    float(closest_point[1]),
                    float(closest_point[2]),
                )
            else:
                self.get_logger().info(f'Target reachable (dist={dist:.3f}m).')

        if self.current_joint_state is None:
            self.get_logger().warn('Waiting for joint states...')
            return

        if not self.ik_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')
            return

        self.get_logger().info(f'Planning to: X={x:.3f}, Y={y:.3f}, Z={z:.3f}')

        request = GetPositionIK.Request()
        request.ik_request.group_name = 'mk2_arm'
        request.ik_request.pose_stamped.header.frame_id = 'arm_base_link'
        request.ik_request.pose_stamped.pose.position = Point(x=x, y=y, z=z)
        # Orientation: Forward/Down
        request.ik_request.pose_stamped.pose.orientation = Quaternion(
            x=0.0, y=0.707, z=0.0, w=0.707
        )

        request.ik_request.robot_state.joint_state.name = self.joint_names
        request.ik_request.robot_state.joint_state.position = list(
            self.current_joint_state.position
        )

        future = self.ik_service.call_async(request)
        future.add_done_callback(
            lambda future: self.ik_response_callback(future, duration_seconds)
        )

    def ik_response_callback(self, future, duration_seconds: float):
        try:
            result = future.result()
            if result.error_code.val == 1:
                joint_angles = result.solution.joint_state.position
                self.execute_trajectory(joint_angles, duration_seconds)
            else:
                self.get_logger().error(f'IK failed with error code: {result.error_code.val}')
        except Exception as e:
            self.get_logger().error(f'IK Service call failed: {e}')

    def execute_trajectory(self, joint_angles, duration_seconds: float):
        traj = JointTrajectory()
        traj.header.frame_id = 'arm_base_link'
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = list(joint_angles)
        point.time_from_start = Duration(seconds=duration_seconds).to_msg()
        traj.points = [point]
        self.trajectory_pub.publish(traj)
        self.get_logger().info(f'Executing trajectory ({duration_seconds}s)')


def main(args=None):
    rclpy.init(args=args)
    controller = CoordinateController()
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    try:
        executor.spin()
    except KeyboardInterrupt as err:
        raise err
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
