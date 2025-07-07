from __future__ import annotations  # compatibility with Python 3.8

import traceback
from itertools import chain
from math import pi

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Int32

import reseq_ros2.constants as rc
from reseq_interfaces.msg import Motors


class State:
    name: str
    value: float

    def __init__(self, name, value) -> None:
        self.name = name
        self.value = value

    def update(self, value):
        self.value = value


"""
ROS node publish Joint states updates which are visible on the Robot Model
in RVIZ or other tools. It is part of the Digital Twin project

It receives feedbacks from the Communication node and calculates the angular
position or velocity. Two subsequent messages are sent: one for the wheels
velocity and one for the joint and end effector joints angular positions
"""


class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.get_logger().info('JointPublisher node started')

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        modules = self.declare_parameter('modules', [0]).get_parameter_value().integer_array_value

        joints = self.declare_parameter('joints', [0]).get_parameter_value().integer_array_value

        end_effector = (
            self.declare_parameter('end_effector', 0).get_parameter_value().integer_value
        )

        self.d = self.declare_parameter('d', 0.0).get_parameter_value().double_value
        self.r_eq = self.declare_parameter('r_eq', 0.0).get_parameter_value().double_value

        self.arm_pitch_origin = (
            self.declare_parameter('arm_pitch_origin', 0).get_parameter_value().integer_value
        )
        self.head_pitch_origin = (
            self.declare_parameter('head_pitch_origin', 0).get_parameter_value().integer_value
        )
        self.head_roll_origin = (
            self.declare_parameter('head_roll_origin', 0).get_parameter_value().integer_value
        )

        self.velocity_gain = (
            self.declare_parameter('vel_gain', 0.0).get_parameter_value().double_value
        )

        self.arm_pitch_gain = (
            self.declare_parameter('arm_pitch_gain', 0.0).get_parameter_value().double_value
        )

        self.wheel_velocities = []
        self.position_states = {}

        self.controller_pubs = []
        self.init_state(modules, joints, end_effector)
        self.create_subs(modules, joints, end_effector)

        self.get_logger().info('States initialized')

        self.create_timer(rc.sample_time, self.broadcast_states)

    def init_state(self, modules: list[int], joints: list[int], end_effector: int):
        """
        Creates the states, given the robot configuration, using a dictionary
        `(module_address, generic_state): State(name=specific_state, value=0.0)`
        in which the specific state is the generic state specialised to the
        current module
        """
        for mod in modules:
            module_id = mod % 16

            self.controller_pubs.append(
                self.create_publisher(TwistStamped, f'/diff_controller{module_id}/cmd_vel', 10)
            )

            self.wheel_velocities.append([0.0, 0.0])

            if mod in joints:
                self.position_states.update(
                    {
                        (mod, x): State(x + f'_{module_id}_joint', 0.0)
                        for x in self.states_from_type(rc.StateType.JOINT_FEEDBACK)
                    }
                )

            if mod == end_effector:
                self.position_states.update(
                    {
                        (mod, x): State(x + f'_{module_id}_joint', 0.0)
                        for x in self.states_from_type(rc.StateType.END_EFFECTOR_FEEDBACK)
                    }
                )

    def create_subs(self, modules: list[int], joints: list[int], end_effector: int):
        """
        Creates the subscription to all the feedback topics linked to Joints
        (the ones in `rc.states`)
        """
        for mod in modules:
            for topic in self.topics_from_type(rc.StateType.MOTOR_FEEDBACK):
                self.create_subscription(
                    Motors,
                    f'reseq/module{mod}/{topic}',
                    lambda msg, addr=mod, tp=topic: self.update_callback(
                        msg, addr, tp, rc.StateType.MOTOR_FEEDBACK
                    ),
                    10,
                )
            if mod in joints:
                for topic in self.topics_from_type(rc.StateType.JOINT_FEEDBACK):
                    self.create_subscription(
                        Float32,
                        f'reseq/module{mod}/{topic}',
                        lambda msg, addr=mod, tp=topic: self.update_callback(
                            msg, addr, tp, rc.StateType.JOINT_FEEDBACK
                        ),
                        10,
                    )
            if mod is end_effector:
                for topic in self.topics_from_type(rc.StateType.END_EFFECTOR_FEEDBACK):
                    self.create_subscription(
                        Int32,
                        f'reseq/module{mod}/{topic}',
                        lambda msg, addr=mod, tp=topic: self.update_callback(
                            msg, addr, tp, rc.StateType.END_EFFECTOR_FEEDBACK
                        ),
                        10,
                    )

    def broadcast_states(self):
        """
        Creates and publishes the `JointState` messages, based on a timer
        of `rc.sample_time` seconds
        """
        # Send wheel velocities for each module
        for i, vel in enumerate(self.wheel_velocities):
            vl = vel[0] * rc.rpm2rads * self.r_eq
            vr = vel[1] * rc.rpm2rads * self.r_eq

            # compute velocity of the module given the feedback velocity of its wheels
            w = (vr - vl) / self.d
            v = (vr + vl) / 2

            # publish Twist to differential controller
            t = Twist()
            t.linear.x = v
            t.angular.z = w
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist = t
            self.controller_pubs[i].publish(msg)

        position = JointState()
        now = self.get_clock().now()
        position.header.stamp = now.to_msg()
        position.name = [x.name for x in self.position_states.values()]
        position.position = [x.value for x in self.position_states.values()]

        self.joint_pub.publish(position)

    def update_callback(
        self,
        msg: Motors | Float32 | Int32,
        address: int,
        topic: str,
        state_t: rc.StateType,
    ):
        """
        Given the module address, the topic suffix, and the state type, this callback receives
        listens on the feedback topics and updates the internal representation of the Joints
        """
        if state_t == rc.StateType.MOTOR_FEEDBACK:
            left, right = msg.left, msg.right

            left *= self.velocity_gain
            right *= self.velocity_gain

            self.wheel_velocities[(address % 16) - 1] = [left, right]

        if state_t == rc.StateType.JOINT_FEEDBACK:
            angle = msg.data

            if angle >= 180:
                angle -= 360

            st = self.states_from_topic(topic)[0]
            self.position_states[(address, st)].update(angle * pi / 180.0)

        if state_t == rc.StateType.END_EFFECTOR_FEEDBACK:
            lsb = msg.data
            st = self.states_from_topic(topic)[0]
            if topic == 'end_effector/pitch/feedback':
                self.position_states[(address, st)].update(
                    (lsb - self.arm_pitch_origin) * rc.lsb_to_rads * self.arm_pitch_gain * (-1)
                )
            if topic == 'end_effector/head_pitch/feedback':
                self.position_states[(address, st)].update(
                    (lsb - self.head_pitch_origin) * rc.lsb_to_rads
                )
            if topic == 'end_effector/head_roll/feedback':
                self.position_states[(address, st)].update(
                    (lsb - self.head_roll_origin) * rc.lsb_to_rads
                )

    def states_from_type(self, state_type: rc.StateType) -> list[str]:
        return list(
            chain.from_iterable([x.states for x in rc.states if x.state_type == state_type])
        )

    def states_from_topic(self, topic: str) -> list[str]:
        return next(map(lambda x: x.states, filter(lambda x: x.topic == topic, rc.states)))

    def topics_from_type(self, state_type: rc.StateType) -> list[str]:
        return [x.topic for x in rc.states if x.state_type == state_type]


def main(args=None):
    rclpy.init(args=args)
    try:
        joint_publisher = JointPublisher()
        rclpy.spin(joint_publisher)
    except Exception as err:
        rclpy.logging.get_logger('joint_publisher').fatal(
            f'Error in the JointPublisher node: {str(err)}\n{traceback.format_exc()}'
        )
        raise err
    else:
        joint_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
