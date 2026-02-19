#!/usr/bin/env python3
import traceback

import rclpy
from geometry_msgs.msg import TwistStamped, Vector3
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from std_srvs.srv import SetBool


class MoveitController(Node):
    """ROS2 node for controlling the arm using MoveIt Servo.

    This node acts as an interface between high-level velocity commands and
    MoveIt Servo. It handles:
    - Converting Vector3 velocity commands to TwistStamped messages
    - Switching between linear and angular velocity control modes
    - Mirroring specific joint states to fix MoveIt issue with non-arm joints
    - Controlling the arm's beak (gripper) mechanism

    Subscribed Topics:
        /mk2_arm_vel (geometry_msgs/Vector3): Velocity commands for the arm
        /joint_states (sensor_msgs/JointState): Current joint states of the robot

    Published Topics:
        /moveit_servo_node/delta_twist_cmds (geometry_msgs/TwistStamped):
            Velocity commands for MoveIt Servo
        /arm_joint_states (sensor_msgs/JointState): Filtered joint states
        reseq/module33/mk2_arm/beak/setpoint (std_msgs/Int32): Beak control commands

    Services:
        /moveit_controller/switch_vel (std_srvs/SetBool):
            Switch between linear and angular velocity modes
        /moveit_controller/close_beak (std_srvs/SetBool):
            Open or close the arm's beak

    Parameters:
        planning_frame_id (str): Frame ID for motion planning (default: 'arm_base_link')
    """

    def __init__(self):
        super().__init__('moveit_controller')

        self.linear_vel_enabled = True
        self.create_service(SetBool, '/moveit_controller/switch_vel', self.switch_vel_type)
        self.create_service(SetBool, '/moveit_controller/close_beak', self.handle_beak)

        self.planning_frame_id = (
            self.declare_parameter('planning_frame_id', 'arm_base_link')
            .get_parameter_value()
            .string_value
        )

        self.create_subscription(Vector3, '/mk2_arm_vel', self.handle_velocities, 10)
        self.speed_pub = self.create_publisher(
            TwistStamped, '/moveit_servo_node/delta_twist_cmds', 10
        )

        self.current_joints = {}

        self.mirror_pub = self.create_publisher(JointState, '/arm_joint_states', 10)
        self.create_subscription(JointState, '/joint_states', self.mirror_states, 10)
        self.state_to_mirror = [
            'mod1__base_pitch_arm_joint',
            'mod1__base_roll_arm_joint',
            'mod1__elbow_pitch_arm_joint',
            'mod1__forearm_roll_arm_joint',
            'mod1__wrist_pitch_arm_joint',
            'mod1__wrist_roll_arm_joint',
        ]

        self.beak_pub = self.create_publisher(  # TODO: Fix with HW interface
            Int32,
            'reseq/module33/mk2_arm/beak/setpoint',
            10,
        )
        self.get_logger().info('Node Moveit Controller started successfully')

    def mirror_states(self, msg: JointState):
        """Mirror specific joint states to a filtered topic.

        Filters the incoming joint states message to only include joints
        specified in self.state_to_mirror and publishes them to a separate topic.
        This fixes issues with MoveIt when non-arm joints are present.

        Args:
            msg (JointState): The incoming joint states message containing all joints.

        Note:
            Updates self.current_joints dictionary with all received joint positions.
        """
        fmsg = JointState()
        fmsg.header = msg.header
        is_to_pub = False
        for i, name in enumerate(msg.name):
            self.current_joints[name] = msg.position[i]
            if name in self.state_to_mirror:
                is_to_pub = True
                fmsg.name.append(name)
                fmsg.position.append(msg.position[i])
                if msg.velocity != []:
                    fmsg.velocity.append(msg.velocity[i])
                if msg.effort != []:
                    fmsg.effort.append(msg.effort[i])
        if is_to_pub:
            self.mirror_pub.publish(fmsg)

    def handle_velocities(self, msg: Vector3):
        """Convert incoming velocity commands to MoveIt Servo format.

        Processes Vector3 velocity commands and converts them into TwistStamped
        messages for MoveIt Servo. The velocity is interpreted as either linear
        or angular based on the current state of self.linear_vel_enabled.

        Args:
            msg (Vector3): Velocity command (x, y, z components).

        Note:
            The frame_id for the twist command is set from the planning_frame_id parameter.
        """
        servo_msg = TwistStamped()
        servo_msg.header.stamp = self.get_clock().now().to_msg()
        servo_msg.header.frame_id = self.planning_frame_id

        if self.linear_vel_enabled:
            servo_msg.twist.linear = msg
        else:
            servo_msg.twist.angular = msg

        self.speed_pub.publish(servo_msg)

    def switch_vel_type(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Service callback to switch between linear and angular velocity modes.

        Args:
            request (SetBool.Request): Service request containing:
                - data (bool): True for linear velocity, False for angular velocity
            response (SetBool.Response): Service response to be filled

        Returns:
            SetBool.Response: Response indicating success and current velocity mode
        """
        self.linear_vel_enabled = request.data

        response.success = True
        response.message = (
            'Input velocity set to LINEAR'
            if self.linear_vel_enabled
            else 'Input velocity set to ANGULAR'
        )
        self.get_logger().info(response.message)
        return response

    def handle_beak(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Service callback to control the arm's beak (gripper) mechanism.

        Args:
            request (SetBool.Request): Service request containing:
                - data (bool): True to close the beak, False to open it
            response (SetBool.Response): Service response to be filled

        Returns:
            SetBool.Response: Response indicating success and the requested action

        Note:
            TODO: This currently uses a direct topic publish but should be
            updated to use the hardware interface.
        """
        self.beak_pub.publish(Int32(data=int(request.data)))
        response.success = True
        response.message = f'Sent request to {"CLOSE" if request.data else "OPEN"} the arm beak'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        moveit_controller = MoveitController()
        rclpy.spin(moveit_controller)
    except KeyboardInterrupt:
        rclpy.logging.get_logger('moveit_controller').warn(
            'Moveit controller node interrupted by user'
        )
    except Exception as err:
        rclpy.logging.get_logger('moveit_controller').fatal(
            f'Error in the Moveit controller node: {str(err)}\n{traceback.format_exc()}'
        )
    else:
        moveit_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
