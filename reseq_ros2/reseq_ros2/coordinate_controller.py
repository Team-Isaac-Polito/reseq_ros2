#!/usr/bin/env python3
"""
Coordinate-based arm controller.
Accepts target coordinates and uses MoveIt IK to move the arm there.

Usage:
    ros2 run reseq_ros2 coordinate_controller
"""

import rclpy
from geometry_msgs.msg import Point, Quaternion
from moveit_msgs.srv import GetPositionIK
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class CoordinateController(Node):
    def __init__(self):
        super().__init__('coordinate_controller')

        self.ik_service = self.create_client(GetPositionIK, '/compute_ik')
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )

        # Subscribe to joint states to get current configuration
        self.current_joint_state = None
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # NEW: Subscribe to a topic to receive target coordinates
        self.create_subscription(Point, '~/move_to_point', self.move_to_point_callback, 10)

        self.joint_names = [
            'mod1__base_pitch_arm_joint',
            'mod1__base_roll_arm_joint',
            'mod1__elbow_pitch_arm_joint',
            'mod1__forearm_roll_arm_joint',
            'mod1__wrist_pitch_arm_joint',
            'mod1__wrist_roll_arm_joint',
        ]

        self.get_logger().info('CoordinateController initialized. Waiting for /joint_states...')

    def joint_state_callback(self, msg: JointState):
        """Store current joint state for use in IK requests"""
        if self.current_joint_state is None:
            self.get_logger().info(
                'First joint state received. Ready to accept commands on ~/move_to_point.'
            )
        self.current_joint_state = msg

    def move_to_point_callback(self, msg: Point):
        """Callback to move to a specific point received from a topic."""
        self.get_logger().info(f'Received new target: X={msg.x}, Y={msg.y}, Z={msg.z}')
        # Call the main logic with a default duration
        self.move_to_coordinate(msg.x, msg.y, msg.z, duration_seconds=5.0)

    def move_to_coordinate(self, x: float, y: float, z: float, duration_seconds: float = 5.0):
        """
        Move arm to a coordinate using inverse kinematics.

        Args:
            x, y, z: Target position in meters (relative to arm_base_link)
            duration_seconds: Time to execute the motion
        """
        # Wait for the first joint state to be received
        if self.current_joint_state is None:
            self.get_logger().warn('Waiting for first /joint_states message. Ignoring command.')
            return

        # Wait for service
        while not self.ik_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')

        self.get_logger().info(f'Planning to coordinate: X={x:.3f}, Y={y:.3f}, Z={z:.3f}')

        # Create IK request
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'mk2_arm'
        request.ik_request.pose_stamped.header.frame_id = 'arm_base_link'
        request.ik_request.pose_stamped.pose.position = Point(x=x, y=y, z=z)
        request.ik_request.pose_stamped.pose.orientation = Quaternion(
            x=0.0, y=0.707, z=0.0, w=0.707
        )

        # Use current state as seed configuration
        if self.current_joint_state is not None:
            request.ik_request.robot_state.joint_state.name = self.joint_names
            # Use actual current positions
            request.ik_request.robot_state.joint_state.position = list(
                self.current_joint_state.position
            )
        else:
            self.get_logger().warn('No joint state received yet, using zeros')
            request.ik_request.robot_state.joint_state.name = self.joint_names
            request.ik_request.robot_state.joint_state.position = [0.0] * len(self.joint_names)

        # Call service asynchronously and attach a callback
        future = self.ik_service.call_async(request)
        # Pass the duration to the callback using a lambda
        future.add_done_callback(
            lambda future: self.ik_response_callback(future, duration_seconds)
        )
        self.get_logger().info('Sent IK request, waiting for solution...')

    def ik_response_callback(self, future, duration_seconds: float):
        """Callback to handle the response from the /compute_ik service."""
        result = future.result()

        if result is None:
            self.get_logger().error('IK service call failed.')
            return

        if result.error_code.val == 1:  # M_SUCCESS
            joint_angles = result.solution.joint_state.position
            self.get_logger().info(f'IK solution found: {joint_angles}')
            self.execute_trajectory(joint_angles, duration_seconds)
        else:
            self.get_logger().error(f'IK failed with error code: {result.error_code.val}')

    def execute_trajectory(self, joint_angles, duration_seconds: float):
        """Send a trajectory to the arm controller"""
        traj = JointTrajectory()
        traj.header.frame_id = 'arm_base_link'
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = list(joint_angles)
        point.time_from_start = Duration(seconds=duration_seconds).to_msg()

        traj.points = [point]
        self.trajectory_pub.publish(traj)
        self.get_logger().info(f'Sent trajectory. Arm will reach target in {duration_seconds}s')


def main(args=None):
    rclpy.init(args=args)
    controller = CoordinateController()

    # Use a MultiThreadedExecutor to prevent deadlocks
    executor = MultiThreadedExecutor()
    executor.add_node(controller)

    try:
        # Spin the node to keep it alive and processing callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
