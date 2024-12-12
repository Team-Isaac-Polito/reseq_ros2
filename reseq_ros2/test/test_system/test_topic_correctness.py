import importlib
import struct
import sys
import time
import unittest
from test.utils.test_utils import check_interface_status, simulate_launch_test, tear_down_process

import can
import geometry_msgs.msg
import launch_testing
import launch_testing.actions
import rclpy
import sensor_msgs.msg
import std_msgs.msg
from geometry_msgs.msg import Vector3
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from rclpy.node import Node

import reseq_interfaces.msg
from reseq_interfaces.msg import Remote


class FrequencyChecker(Node):
    """
    A ROS2 node that checks the frequency of messages on specified topics.

    This node subscribes to multiple topics and calculates the frequency of the received messages.
    It also publishes messages to a specified topic at a given rate.
    """

    def __init__(self, topics, string_to_msg_type, pub, msg, canbus):
        super().__init__('frequency_checker')

        self.subscriptions_ = {}
        self.intervals = {}
        self.pub_ = pub
        self.msg = msg
        self.canbus = canbus

        for topic_name, msg_type in topics:
            self.subscriptions_[topic_name] = self.create_subscription(
                string_to_msg_type(msg_type[0]), topic_name, self.create_callback(topic_name), 10
            )
            self.intervals[topic_name] = {'last_time': None, 'intervals': []}

        self.timer1 = self.create_timer(0.08, self.publisher_callback)  # 12.5 Hz
        if self.canbus:
            self.timer2 = self.create_timer(0.01, self.feedback_callback)  # 100 Hz

    def create_callback(self, topic_name):
        """Create a callback function to calculate the intervals between messages."""

        def callback(msg):
            current_time = time.time()
            last_time = self.intervals[topic_name]['last_time']
            if last_time is not None:
                interval = current_time - last_time
                self.intervals[topic_name]['intervals'].append(interval)
                if len(self.intervals[topic_name]['intervals']) > 100:
                    self.intervals[topic_name]['intervals'].pop(0)
            self.intervals[topic_name]['last_time'] = current_time

        return callback

    def publisher_callback(self):
        """Publish messages at the specified rate."""
        self.pub_.publish(self.msg)

    def feedback_callback(self):
        """Send CAN messages to trigger the feedback callback at the specified rate."""
        for mod_id, module_num, data in [
            (0x22, 0x11, struct.pack('ff', 5.0, 3.0)),
            (0x22, 0x12, struct.pack('ff', 5.0, 3.0)),
            (0x22, 0x13, struct.pack('ff', 5.0, 3.0)),
            (0x32, 0x12, struct.pack('f', 5.0)),
            (0x34, 0x12, struct.pack('f', 5.0)),
            (0x36, 0x12, struct.pack('f', 5.0)),
            (0x32, 0x13, struct.pack('f', 5.0)),
            (0x34, 0x13, struct.pack('f', 5.0)),
            (0x36, 0x13, struct.pack('f', 5.0)),
            (0x42, 0x11, struct.pack('i', 5)),
            (0x44, 0x11, struct.pack('i', 5)),
            (0x46, 0x11, struct.pack('i', 5)),
        ]:
            # Create a CAN message
            aid = struct.pack('bbbb', 00, mod_id, 0x00, module_num)
            msg = can.Message(
                arbitration_id=int.from_bytes(aid, byteorder='big', signed=False),
                data=data,
                is_extended_id=True,
            )
            # Send the message to trigger the can_callback
            self.canbus.send(msg)

    def get_frequency(self):
        """Calculate the frequency of messages for each topic."""
        freqs = {}
        for topic_name, data in self.intervals.items():
            intervals = data['intervals']
            if intervals:
                avg_interval = sum(intervals) / len(intervals)
                frequency = 1.0 / avg_interval if avg_interval > 0 else 0
                freqs[topic_name] = frequency
        return freqs


class TestNodes(unittest.TestCase):
    """
    Unit test class for testing ROS2 nodes.

    This class contains tests to verify the accuracy and frequency of messages
    published and subscribed by ROS2 nodes.
    """

    # Define stat as a class attribute
    stat = check_interface_status('vcan0')

    @classmethod
    def setUpClass(cls):
        cls.process = simulate_launch_test(sys.argv[0])
        rclpy.init()
        cls.node = rclpy.create_node('test_node')
        cls.pub_ = cls.node.create_publisher(Remote, '/remote', 10)
        cls.msg = Remote()
        cls.msg.left = Vector3(x=0.5, y=-0.2, z=1.0)
        cls.msg.right = Vector3(x=-0.8, y=0.3, z=0.7)

        # Initialize the CAN bus interface
        if cls.stat[0]:
            cls.canbus = can.interface.Bus(channel='vcan0', bustype='socketcan')

        # Wait for launch
        time.sleep(5)

    @classmethod
    def tearDownClass(cls):
        tear_down_process(cls.process)
        if cls.stat[0]:
            cls.canbus.shutdown()
        rclpy.shutdown()

    def setUp(self):
        self.msgs = [None] * 32

    def tearDown(self):
        for sub in self.node.subscriptions:
            self.node.destroy_subscription(sub)

    def test_1_forward(self):
        """Test the message accuracy of the publisher topics."""
        # Create the subscribers for each topic
        if not self.stat[0] and self.stat[1].split()[-1] == 'exist.':
            print('Skip the message accuracy test for setpoint topics')
            n_topic = 3
            topics = [
                (topic, topic_type)
                for topic, topic_type in self.node.get_topic_names_and_types()
                if topic in ['/cmd_vel', '/end_effector', '/remote']
            ]
        else:
            n_topic = 9
            topics = [
                (topic, topic_type)
                for topic, topic_type in self.node.get_topic_names_and_types()
                if len(self.node.get_subscriptions_info_by_topic(topic)) != 0
                and 'joint' not in topic.split('/')
                and topic.split('/')[-1] != 'feedback'
                and topic
                not in [
                    '/joint_states',
                    '/diff_controller1/cmd_vel',
                    '/diff_controller2/cmd_vel',
                    '/diff_controller3/cmd_vel',
                    '/robot_description',
                    '/parameter_events',
                ]
            ]
        topic_names = []
        idx = 0
        for topic, topic_type in topics:
            self.node.create_subscription(
                self.string_to_msg_type(topic_type[0]),
                topic,
                self.create_callback(idx, topic_type),
                10,
            )
            topic_names.append(topic)
            idx += 1

        # Wait until it transmits message
        endtime = time.time() + 10
        while time.time() < endtime:
            self.pub_.publish(self.msg)
            rclpy.spin_once(self.node, timeout_sec=0)
            if None not in self.msgs[:idx]:
                break

        # Test accuracy
        expected = {
            '/cmd_vel': geometry_msgs.msg.Twist(
                linear=geometry_msgs.msg.Vector3(x=0.08105100000000004, y=0.0, z=0.0),
                angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.09975757080000004),
            ),
            '/end_effector': reseq_interfaces.msg.EndEffector(
                pitch_vel=-36.599998474121094, head_pitch_vel=400.0, head_roll_vel=227.5
            ),
            '/remote': reseq_interfaces.msg.Remote(
                left=geometry_msgs.msg.Vector3(x=0.5, y=-0.2, z=1.0),
                right=geometry_msgs.msg.Vector3(x=-0.8, y=0.3, z=0.7),
                buttons=[],
                switches=[],
            ),
            '/reseq/module17/end_effector/head_pitch/setpoint': std_msgs.msg.Int32(data=800),
            '/reseq/module17/end_effector/head_roll/setpoint': std_msgs.msg.Int32(data=0),
            '/reseq/module17/end_effector/pitch/setpoint': std_msgs.msg.Int32(data=750),
            '/reseq/module17/motor/setpoint': reseq_interfaces.msg.Motors(
                left=14.360505104064941, right=18.928958892822266
            ),
            '/reseq/module18/motor/setpoint': reseq_interfaces.msg.Motors(
                left=18.749523162841797, right=14.53994083404541
            ),
            '/reseq/module19/motor/setpoint': reseq_interfaces.msg.Motors(
                left=14.705280303955078, right=18.584184646606445
            ),
        }

        self.assertTrue(
            idx == n_topic,
            f"The number of subscribers doesn't match. Expected {n_topic}, got {idx}",
        )
        for msg, topic in zip(self.msgs[:idx], topic_names):
            exp = expected[topic]
            self.assertEqual(
                msg, exp, f'The message of {topic} is incorrect. Expected {exp}, got {msg}'
            )

    @unittest.skipUnless(stat[0], stat[1])
    def test_2_feedback(self):
        """Test the message accuracy of the telemetry/feedback."""
        # Create the subscribers for each topic
        topics = [
            (topic, topic_type)
            for topic, topic_type in self.node.get_topic_names_and_types()
            if topic.split('/')[-1] == 'feedback'
            or topic
            in [
                '/joint_states',
                '/diff_controller1/cmd_vel',
                '/diff_controller2/cmd_vel',
                '/diff_controller3/cmd_vel',
            ]
        ]
        topic_names = []
        idx = 0
        for topic, topic_type in topics:
            self.node.create_subscription(
                self.string_to_msg_type(topic_type[0]),
                topic,
                self.create_callback(idx, topic_type),
                10,
            )
            topic_names.append(topic)
            idx += 1

        # Wait until it transmits message
        endtime = time.time() + 10
        while time.time() < endtime:
            for mod_id, module_num, data in [
                (0x22, 0x11, struct.pack('ff', 5.0, 3.0)),
                (0x22, 0x12, struct.pack('ff', 5.0, 3.0)),
                (0x22, 0x13, struct.pack('ff', 5.0, 3.0)),
                (0x32, 0x12, struct.pack('f', 5.0)),
                (0x34, 0x12, struct.pack('f', 5.0)),
                (0x36, 0x12, struct.pack('f', 5.0)),
                (0x32, 0x13, struct.pack('f', 5.0)),
                (0x34, 0x13, struct.pack('f', 5.0)),
                (0x36, 0x13, struct.pack('f', 5.0)),
                (0x42, 0x11, struct.pack('i', 5)),
                (0x44, 0x11, struct.pack('i', 5)),
                (0x46, 0x11, struct.pack('i', 5)),
            ]:
                # Create a CAN message
                aid = struct.pack('bbbb', 00, mod_id, 0x00, module_num)
                msg = can.Message(
                    arbitration_id=int.from_bytes(aid, byteorder='big', signed=False),
                    data=data,
                    is_extended_id=True,
                )
                # Send the message to trigger the can_callback
                self.canbus.send(msg)
            rclpy.spin_once(self.node, timeout_sec=0)
            if None not in self.msgs[:idx]:
                break

        # Test accuracy
        expected = {
            '/diff_controller1/cmd_vel': geometry_msgs.msg.TwistStamped(
                twist=geometry_msgs.msg.Twist(
                    linear=geometry_msgs.msg.Vector3(x=2.10088, y=0.0, z=0.0),
                    angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=-5.7104648002174505),
                )
            ),
            '/diff_controller2/cmd_vel': geometry_msgs.msg.TwistStamped(
                twist=geometry_msgs.msg.Twist(
                    linear=geometry_msgs.msg.Vector3(x=2.10088, y=0.0, z=0.0),
                    angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=-5.7104648002174505),
                )
            ),
            '/diff_controller3/cmd_vel': geometry_msgs.msg.TwistStamped(
                twist=geometry_msgs.msg.Twist(
                    linear=geometry_msgs.msg.Vector3(x=2.10088, y=0.0, z=0.0),
                    angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=-5.7104648002174505),
                )
            ),
            '/joint_states': sensor_msgs.msg.JointState(
                name=[
                    'arm_pitch_1_joint',
                    'arm_head_pitch_1_joint',
                    'arm_head_roll_1_joint',
                    'joint_y_2_joint',
                    'joint_p_2_joint',
                    'joint_r_2_joint',
                    'joint_y_3_joint',
                    'joint_p_3_joint',
                    'joint_r_3_joint',
                ],
                position=[
                    0.508446035955,
                    -4.0690485,
                    -2.5949781,
                    0.08726646259971647,
                    0.08726646259971647,
                    0.08726646259971647,
                    0.08726646259971647,
                    0.08726646259971647,
                    0.08726646259971647,
                ],
                velocity=[],
                effort=[],
            ),
            '/reseq/module17/end_effector/head_pitch/feedback': std_msgs.msg.Int32(data=5),
            '/reseq/module17/end_effector/head_roll/feedback': std_msgs.msg.Int32(data=5),
            '/reseq/module17/end_effector/pitch/feedback': std_msgs.msg.Int32(data=5),
            '/reseq/module17/motor/feedback': reseq_interfaces.msg.Motors(left=5.0, right=3.0),
            '/reseq/module18/joint/pitch/feedback': std_msgs.msg.Float32(data=5.0),
            '/reseq/module18/joint/roll/feedback': std_msgs.msg.Float32(data=5.0),
            '/reseq/module18/joint/yaw/feedback': std_msgs.msg.Float32(data=5.0),
            '/reseq/module18/motor/feedback': reseq_interfaces.msg.Motors(left=5.0, right=3.0),
            '/reseq/module19/joint/pitch/feedback': std_msgs.msg.Float32(data=5.0),
            '/reseq/module19/joint/roll/feedback': std_msgs.msg.Float32(data=5.0),
            '/reseq/module19/joint/yaw/feedback': std_msgs.msg.Float32(data=5.0),
            '/reseq/module19/motor/feedback': reseq_interfaces.msg.Motors(left=5.0, right=3.0),
        }

        self.assertTrue(
            len(self.msgs[:idx]) == len(expected),
            (
                f"The number of subscribers doesn't match. "
                f'Expected {len(expected)}, got {len(self.msgs[:idx])}'
            ),
        )
        for msg, topic in zip(self.msgs[:idx], topic_names):
            exp = expected[topic]
            if topic in ['/joint_states']:
                self.assertEqual(
                    msg.position,
                    exp.position,
                    (
                        f'The message of {topic} is incorrect. '
                        f'Expected {exp.position}, got {msg.position}'
                    ),
                )
                continue
            if topic in [
                '/diff_controller1/cmd_vel',
                '/diff_controller2/cmd_vel',
                '/diff_controller3/cmd_vel',
            ]:
                self.assertEqual(
                    msg.twist,
                    exp.twist,
                    f'The message of {topic} is incorrect. Expected {exp.twist}, got {msg.twist}',
                )
                continue
            self.assertEqual(
                msg, exp, f'The message of {topic} is incorrect. Expected {exp}, got {msg}'
            )

    def test_3_frequency(self):
        """Test the frequency accuracy."""
        if not self.stat[0]:
            print('Skip the message frequency test for feedback topics')
            self.canbus = None
            n_topic = 9
        else:
            n_topic = 21
        topics = [
            (topic, topic_type)
            for topic, topic_type in self.node.get_topic_names_and_types()
            if (topic.split('/')[-1] == 'feedback' and self.stat[0])
            or topic
            in [
                '/cmd_vel',
                '/end_effector',
                '/remote',
                '/reseq/module17/end_effector/head_pitch/setpoint',
                '/reseq/module17/end_effector/head_roll/setpoint',
                '/reseq/module17/end_effector/pitch/setpoint',
                '/reseq/module17/motor/setpoint',
                '/reseq/module18/motor/setpoint',
                '/reseq/module19/motor/setpoint',
            ]
        ]
        node = FrequencyChecker(topics, self.string_to_msg_type, self.pub_, self.msg, self.canbus)

        # Wait to gather frequency info
        endtime = time.time() + 1
        while time.time() < endtime:
            rclpy.spin_once(node, timeout_sec=0)

        # Test accuracy
        freqs = node.get_frequency()
        self.assertTrue(
            len(topics) == n_topic,
            f"The number of subscribers doesn't match. Expected {n_topic}, got {len(topics)}",
        )
        for topic, frequency in freqs.items():
            if topic.split('/')[-1] == 'feedback':
                self.assertAlmostEqual(
                    frequency,
                    100,
                    delta=2,
                    msg=(
                        f'The frequency of {topic} is incorrect. '
                        f'Expected {100}±2, got {frequency}'
                    ),
                )
                continue
            else:
                self.assertAlmostEqual(
                    frequency,
                    12.5,
                    delta=0.5,
                    msg=(
                        f'The frequency of {topic} is incorrect. '
                        f'Expected {12.5}±0.5, got {frequency}'
                    ),
                )

        node.destroy_node()

    def create_callback(self, index, topic_type=None):
        twist = geometry_msgs.msg.Twist(
            linear=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0),
            angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0),
        )
        name = [
            'left_back_wheel_1_joint',
            'right_back_wheel_1_joint',
            'right_front_wheel_2_joint',
            'left_back_wheel_2_joint',
            'right_back_wheel_2_joint',
            'right_front_wheel_1_joint',
            'left_front_wheel_2_joint',
            'left_front_wheel_3_joint',
            'right_front_wheel_3_joint',
            'left_front_wheel_1_joint',
            'left_back_wheel_3_joint',
            'right_back_wheel_3_joint',
        ]
        position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        def callback(msg):
            if topic_type == ['geometry_msgs/msg/TwistStamped']:
                if msg.twist != twist:
                    self.msgs[index] = msg
            elif topic_type == ['sensor_msgs/msg/JointState']:
                if msg.name != name and all(
                    [m != n for m, n in zip(msg.position.tolist(), position)]
                ):
                    self.msgs[index] = msg
            else:
                self.msgs[index] = msg

        return callback

    def string_to_msg_type(self, type_str):
        # Split the string to get the package, module, and class
        package, module, class_name = type_str.split('/')
        full_module_name = f'{package}.{module}'

        # Dynamically import the module
        module = importlib.import_module(full_module_name)

        # Get the class from the module
        msg_type = getattr(module, class_name)

        return msg_type


def generate_test_description():
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'launch',
                    'reseq_ros2',
                    'reseq_launch.py',
                    'config_file:=reseq_mk1_vcan.yaml',
                ],
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        'TestNodes': TestNodes,
    }


if __name__ == '__main__':
    unittest.main()
