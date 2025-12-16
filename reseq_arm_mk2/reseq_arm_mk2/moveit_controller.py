#!/usr/bin/env python3
import traceback

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped, Vector3
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray, Int32
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveitController(Node):
    def __init__(self):
        super().__init__('moveit_controller')
        self.arm_module_address = (
            self.declare_parameter('arm_module_address', 0).get_parameter_value().integer_value
        )

        self.linear_vel_enabled = True
        self.create_service(SetBool, '/moveit_controller/switch_vel', self.switch_vel_type)
        self.create_service(SetBool, '/moveit_controller/close_beak', self.handle_beak)
        self.create_service(SetBool, '/moveit_controller/home_pose', self.send_home_pose)

        self.planning_frame_id = (
            self.declare_parameter('planning_frame_id', 'arm_base_link')
            .get_parameter_value()
            .string_value
        )

        self.create_subscription(Vector3, '/mk2_arm_vel', self.handle_velocities, 10)

        servo_twist_topic = '/moveit_servo_node/delta_twist_cmds'
        servo_jog_topic = '/moveit_servo_node/delta_joint_cmds'
        self.speed_pub = self.create_publisher(TwistStamped, servo_twist_topic, 10)
        self.joints_pos_pub = self.create_publisher(JointJog, servo_jog_topic, 10)

        self.servo_client = self.create_client(
            ServoCommandType, '/moveit_servo_node/switch_command_type'
        )

        self.traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )

        self.current_joints = {}
        self.init_timer = self.create_timer(1.0, self.init_sequence)

        self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self.send_joint_trajectory,
            10,
        )

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

        self.pubs = {}
        addr = self.arm_module_address
        self.pubs['mod1__base_pitch_arm_joint_and_roll_arm_joint'] = self.create_publisher(
            Float32MultiArray,
            f'reseq/module{addr}/mk2_arm/mod1__base_pitch_arm_joint_and_roll_arm_joint/setpoint',
            10,
        )
        self.pubs['mod1__elbow_pitch_arm_joint'] = self.create_publisher(
            Float32,
            f'reseq/module{addr}/mk2_arm/mod1__elbow_pitch_arm_joint/setpoint',
            10,
        )
        self.pubs['mod1__forearm_roll_arm_joint'] = self.create_publisher(
            Float32,
            f'reseq/module{addr}/mk2_arm/mod1__forearm_roll_arm_joint/setpoint',
            10,
        )
        self.pubs['mod1__wrist_pitch_arm_joint'] = self.create_publisher(
            Float32,
            f'reseq/module{addr}/mk2_arm/mod1__wrist_pitch_arm_joint/setpoint',
            10,
        )
        self.pubs['mod1__wrist_roll_arm_joint'] = self.create_publisher(
            Float32,
            f'reseq/module{addr}/mk2_arm/mod1__wrist_roll_arm_joint/setpoint',
            10,
        )
        self.beak_pub = self.create_publisher(
            Int32,
            f'reseq/module{addr}/mk2_arm/beak/setpoint',
            10,
        )
        self.get_logger().info('Node Moveit Controller started successfully')

    def init_sequence(self):
        """
        Startup Routine:
        1. Robot starts at 0.0 (Singularity). MoveIt Servo cannot solve IK here.
        2. We must physically move the arm using the JointTrajectoryController.
        3. We CANNOT use Float32 publishers (Controller ignores them).
        4. We MUST specify v=0.0, a=0.0 in the trajectory, or the controller rejects it as unsafe.
        5. We wait for joint_state feedback to confirm the elbow is bent > 0.5 rad.
        6. Only THEN do we switch Servo to TWIST mode.
        """
        if self.traj_pub.get_subscription_count() == 0:
            self.get_logger().info('Initializing: Waiting for arm_controller connection...')
            return

        elbow_pos = self.current_joints.get('mod1__elbow_pitch_arm_joint', 0.0)

        if abs(elbow_pos) < 0.5:
            self.get_logger().warn(
                f'Initializing: Robot still in singularity (Elbow: {elbow_pos:.2f}). '
                'Sending Setup Trajectory...'
            )
            msg = JointTrajectory()
            msg.joint_names = [
                'mod1__base_pitch_arm_joint',
                'mod1__base_roll_arm_joint',
                'mod1__elbow_pitch_arm_joint',
                'mod1__forearm_roll_arm_joint',
                'mod1__wrist_pitch_arm_joint',
                'mod1__wrist_roll_arm_joint',
            ]
            point = JointTrajectoryPoint()
            point.positions = [0.0, 0.0, 1.0, 0.0, 1.0, 0.0]
            point.velocities = [0.0] * 6
            point.accelerations = [0.0] * 6
            point.time_from_start.sec = 2
            msg.points.append(point)
            self.traj_pub.publish(msg)
        else:
            if self.servo_client.service_is_ready():
                req = ServoCommandType.Request()
                req.command_type = ServoCommandType.Request.TWIST
                self.servo_client.call_async(req)
                self.get_logger().info(
                    'Initializing: Robot is bent. Servo switched to TWIST mode.'
                )
                self.init_timer.cancel()

    def mirror_states(self, msg: JointState):
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
        # if self.init_timer.is_canceled():
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
        self.beak_pub.publish(Int32(data=int(request.data)))
        response.success = True
        response.message = f'Sent request to {"CLOSE" if request.data else "OPEN"} the arm beak'
        self.get_logger().info(response.message)
        return response

    def send_home_pose(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.JOINT_JOG
        self.servo_client.call_async(req)

        servo_msg = JointJog()
        servo_msg.header.stamp = self.get_clock().now().to_msg()
        servo_msg.header.frame_id = self.planning_frame_id
        servo_msg.joint_names = [
            'mod1__base_pitch_arm_joint',
            'mod1__base_roll_arm_joint',
            'mod1__elbow_pitch_arm_joint',
            'mod1__forearm_roll_arm_joint',
            'mod1__wrist_pitch_arm_joint',
            'mod1__wrist_roll_arm_joint',
        ]
        servo_msg.displacements = [0.0] * 6
        servo_msg.velocities = [0.05] * 6
        servo_msg.duration = 0.01
        self.joints_pos_pub.publish(servo_msg)

        req_back = ServoCommandType.Request()
        req_back.command_type = ServoCommandType.Request.TWIST
        self.servo_client.call_async(req_back)

        response.success = True
        response.message = 'Moving arm to HOME position'
        self.get_logger().info(response.message)
        return response

    def send_joint_trajectory(self, msg: JointTrajectory):
        update_diff = True
        names = msg.joint_names
        positions = msg.points[len(msg.points) - 1].positions

        for i in range(0, len(names)):
            if names[i] == 'mod1__base_pitch_arm_joint' or names[i] == 'mod1__base_roll_arm_joint':
                if update_diff:
                    i0 = self.find_index_by_name(names, 'mod1__base_pitch_arm_joint')
                    i1 = self.find_index_by_name(names, 'mod1__base_roll_arm_joint')
                    update_diff = False
                    self.pubs['mod1__base_pitch_arm_joint_and_roll_arm_joint'].publish(
                        Float32MultiArray(data=[positions[i0], positions[i1]])
                    )
            else:
                self.pubs[names[i]].publish(Float32(data=positions[i]))

    def find_index_by_name(self, joints, joint_name):
        for i in range(0, len(joints)):
            if joints[i] == joint_name:
                return i


def main(args=None):
    rclpy.init(args=args)
    try:
        moveit_controller = MoveitController()
        rclpy.spin(moveit_controller)
    except Exception as err:
        rclpy.logging.get_logger('moveit_controller').fatal(
            f'Error in the Moveit controller node: {str(err)}\n{traceback.format_exc()}'
        )
        raise err
    else:
        moveit_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
