import struct
import subprocess
import time
import unittest

import can
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Int32

from reseq_interfaces.msg import EndEffector, Motors, Remote


def check_interface_status(interface_name):
    """Check if the specified network interface is up and running."""
    # Run the `ip` command to get interface details
    result = subprocess.run(['ip', 'link', 'show', interface_name], capture_output=True, text=True)

    # Check if the command was successful
    if result.returncode != 0:
        return False, f"Failed to get interface status. Error: {result.stderr}"

    # Check if the interface is up
    if 'UP,LOWER_UP' in result.stdout:
        return True, f"Interface {interface_name} is up and running."
    else:
        return False, f"Interface {interface_name} is not up."


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
        rclpy.init()
        cls.node = rclpy.create_node('test_node')
        cls.pub_ = cls.node.create_publisher(
            Remote,
            '/remote',
            10
        )
        cls.msg = Remote()
        cls.msg.left = Vector3(x=0.5, y=-0.2, z=1.0)
        cls.msg.right = Vector3(x=-0.8, y=0.3, z=0.7)

        cls.address = 17

        cls.module_num = 3
        cls.joint_num = 6
        cls.end_effector_num = 3

        cls.motor_type = Motors
        cls.joint_type = Float32
        cls.end_effector_type = Int32
        cls.diff_controller_type = TwistStamped

        # Initialize the CAN bus interface
        cls.canbus = can.interface.Bus(channel='vcan0', bustype='socketcan')

    @classmethod
    def tearDownClass(cls):
        cls.canbus.shutdown()
        rclpy.shutdown()

    def setUp(self):
        self.topic_names, self.topic_types = map(list, zip(*self.node.get_topic_names_and_types()))
        self.msgs = [None] * 16

    def tearDown(self):
        for sub in self.node.subscriptions:
            self.node.destroy_subscription(sub)

    def test_1_scaler_topics(self):
        """Test the message reception from the scaler topics."""
        self.node.create_subscription(
            EndEffector,
            '/end_effector',
            self.create_callback(0),
            10
        )
        self.node.create_subscription(
            Twist,
            '/cmd_vel',
            self.create_callback(1),
            10
        )

        # Wait until it transmits message
        endtime = time.time() + 10
        while time.time() < endtime:
            self.pub_.publish(self.msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.msgs[0] and self.msgs[1]:
                break

        problems = []
        if not bool(self.msgs[0]):
            problems.append("Couldn't get message from '/end_effector' topic.")
        if not bool(self.msgs[1]):
            problems.append("Couldn't get message from '/cmd_vel' topic.")
        self.assertTrue(len(problems) == 0, f"\n{''.join(problems)}")
        time.sleep(2)  # Wait until launch ends

    def test_2_agevar_topics(self):
        """Test the message reception from the motor setpoint topics."""
        idx = 0
        while 1:
            topic = f"/reseq/module{self.address + idx}/motor/setpoint"
            if topic not in self.topic_names:
                break

            # Test topic type
            arg = self.topic_names.index(topic)
            expected_type_str = f"{self.motor_type.__module__.split('._')[0]}._{self.motor_type.__name__}"
            topic_type_str = self.topic_types[arg][0].replace('msg/', 'msg._').replace('/', '.')
            self.assertEqual(topic_type_str, expected_type_str, f"The type of the topic {topic} is {topic_type_str}. Expected {expected_type_str}")

            self.node.create_subscription(
                self.motor_type,
                topic,
                self.create_callback(idx),
                10
            )
            idx += 1

        # Wait until it transmits message
        endtime = time.time() + 10
        while time.time() < endtime:
            self.pub_.publish(self.msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if None not in self.msgs[:idx]:
                break

        problems = []
        if len(self.msgs[:idx]) != self.module_num:
            problems.append(f"Number of topics doesn't match. Expected {self.module_num}, got {len(self.msgs[:idx])}.")
        for i in range(idx):
            if not bool(self.msgs[i]):
                problems.append(f"Couldn't get message from '/reseq/module{self.address + i}/motor/setpoint' topic.")
        self.assertTrue(len(problems) == 0, f"\n{''.join(problems)}")

    def test_3_enea_topics(self):
        """Test the message reception from the end effector setpoint topics."""
        idx = 0
        for vel in ['pitch', 'head_pitch', 'head_roll']:
            n = 0
            while 1:
                topic = f"/reseq/module{self.address + n}/end_effector/{vel}/setpoint"
                if topic not in self.topic_names:
                    break

                # Test topic type
                arg = self.topic_names.index(topic)
                expected_type_str = f"{self.end_effector_type.__module__.split('._')[0]}._{self.end_effector_type.__name__}"
                topic_type_str = self.topic_types[arg][0].replace('msg/', 'msg._').replace('/', '.')
                self.assertEqual(topic_type_str, expected_type_str, f"The type of the topic {topic} is {topic_type_str}. Expected {expected_type_str}")
                
                self.node.create_subscription(
                    self.end_effector_type,
                    topic,
                    self.create_callback(idx),
                    10
                )
                n += 1
                idx += 1

        # Wait until it transmits message
        endtime = time.time() + 10
        while time.time() < endtime:
            self.pub_.publish(self.msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if None not in self.msgs[:idx]:
                break

        problems = []
        if len(self.msgs[:idx]) != self.end_effector_num:
            problems.append(f"Number of topics doesn't match. Expected {self.end_effector_num}, got {len(self.msgs[:idx])}.")
        for i in range(idx):
            if not bool(self.msgs[i]):
                problems.append(f"Couldn't get message from '/reseq/module{self.address + (i % (idx//3))}/end_effector/{['pitch', 'head_pitch', 'head_roll'][i // (idx//3)]}/setpoint' topic.")
        self.assertTrue(len(problems) == 0, f"\n{''.join(problems)}")

    @unittest.skipUnless(stat[0], stat[1])
    def test_4_communication_topics(self):
        """Test the message reception from various feedback topics."""
        idx = 0
        n_motor = -1
        n_joint = -1
        n_end_effector = -1
        for st in ['motor', 'joint', 'end_effector']:
            while 1:
                if st == 'motor':
                    topic = f"/reseq/module{self.address + idx}/{st}/feedback"
                    topic_type = self.motor_type
                    n_motor += 1
                elif st == 'joint':
                    topic = f"/reseq/module{self.address+1 + ((idx-n_motor)//3)}/{st}/{['pitch', 'roll', 'yaw'][(idx-n_motor)%3]}/feedback"
                    topic_type = self.joint_type
                    n_joint += 1
                else:
                    topic = f"/reseq/module{self.address + ((idx-n_motor-n_joint)//3)}/{st}/{['pitch', 'head_pitch', 'head_roll'][(idx-n_motor-n_joint)%3]}/feedback"
                    topic_type = self.end_effector_type
                    n_end_effector += 1
                if topic not in self.topic_names:
                    break

                # Test topic type
                arg = self.topic_names.index(topic)
                expected_type_str = f"{topic_type.__module__.split('._')[0]}._{topic_type.__name__}"
                topic_type_str = self.topic_types[arg][0].replace('msg/', 'msg._').replace('/', '.')
                self.assertEqual(topic_type_str, expected_type_str, f"The type of the topic {topic} is {topic_type_str}. Expected {expected_type_str}")

                self.node.create_subscription(
                    topic_type,
                    topic,
                    self.create_callback(idx),
                    10
                )
                idx += 1

        # Wait until it transmits message
        endtime = time.time() + 10
        while time.time() < endtime:
            # Loop through various IDs, module numbers, and data
            for id, module_num, data in [(0x22, 0x11, struct.pack('ff', 5.0, 3.0)),
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
                                         (0x46, 0x11, struct.pack('i', 5)),]:
                # Create a CAN message
                aid = struct.pack('bbbb', 00, id, 0x00, module_num)
                msg = can.Message(
                    arbitration_id=int.from_bytes(aid, byteorder='big', signed=False),
                    data=data,
                    is_extended_id=True)
                # Send the message to trigger the can_callback
                self.canbus.send(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if None not in self.msgs[:idx]:
                break

        problems = []
        if n_motor != self.module_num:
            problems.append(f"Number of motors doesn't match. Expected {self.module_num}, got {n_motor}.")
        if n_joint != self.joint_num:
            problems.append(f"Number of joints doesn't match. Expected {self.joint_num}, got {n_joint}.")
        if n_end_effector != self.end_effector_num:
            problems.append(f"Number of end effectors doesn't match. Expected {self.end_effector_num}, got {n_end_effector}.")
        for i in range(idx):
            if n_motor > i: 
                if not bool(self.msgs[i]):
                    problems.append(f"Couldn't get message from '/reseq/module{self.address + i}/motor/feedback' topic." + "\n")
            elif n_motor + n_joint > i:
                if not bool(self.msgs[i]):
                    problems.append(f"Couldn't get message from '/reseq/module{self.address+1 + ((i-n_motor)//3)}/joint/{['pitch', 'roll', 'yaw'][(i-n_motor)%3]}/feedback' topic." + "\n")
            else:
                if not bool(self.msgs[i]):
                    problems.append(f"Couldn't get message from '/reseq/module{self.address}/end_effector/{['pitch', 'head_pitch', 'head_roll'][(i-n_motor-n_joint)%3]}/feedback' topic." + "\n")
        self.assertTrue(len(problems) == 0, f"\n{''.join(problems)}")

    def test_5_jointpublisher_topics(self):
        """Test the message reception from the joint states and diff controller topics."""
        self.node.create_subscription(
            JointState,
            '/joint_states',
            self.create_callback(0),
            10
        )
        idx = self.module_num + 1
        for i in range(1, idx):
            topic = f"/diff_controller{i}/cmd_vel"

            # Test topic type
            arg = self.topic_names.index(topic)
            expected_type_str = f"{self.diff_controller_type.__module__.split('._')[0]}._{self.diff_controller_type.__name__}"
            topic_type_str = self.topic_types[arg][0].replace('msg/', 'msg._').replace('/', '.')
            self.assertEqual(topic_type_str, expected_type_str, f"The type of the topic {topic} is {topic_type_str}. Expected {expected_type_str}")

            self.node.create_subscription(
                self.diff_controller_type,
                topic,
                self.create_callback(i),
                10
            )

        # Wait until it transmits message
        endtime = time.time() + 10
        while time.time() < endtime:
            self.pub_.publish(self.msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if None not in self.msgs[:idx]:
                break

        problems = []
        if not bool(self.msgs[0]):
            problems.append("Couldn't get message from '/joint_states' topic.")
        for i in range(1, idx):
            if not bool(self.msgs[i]):
                problems.append(f"Couldn't get message from '/diff_controller{i}/cmd_vel' topic.")
        self.assertTrue(len(problems) == 0, f"\n{''.join(problems)}")

    def create_callback(self, index):
        def callback(msg):
            self.msgs[index] = msg
        return callback

@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'reseq_ros2', 'reseq_launch.py', 'config_file:=reseq_mk1_vcan.yaml'],
        ),
        launch_testing.actions.ReadyToTest()
    ]), {
        'TestNodes': TestNodes,
    }

if __name__ == '__main__':
    unittest.main()
