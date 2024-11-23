import unittest
import rclpy

from reseq_interfaces.msg import Remote, EndEffector, Motors
from geometry_msgs.msg import Vector3, Twist, TwistStamped
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import JointState
import time

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_testing
import launch_testing.actions

class TestNodes(unittest.TestCase):
    # 1. TODO: get values from .yaml files.
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_node')
        cls.pub_ = cls.node.create_publisher(
            Remote,
            "/remote",
            10
        )
        cls.msg = Remote()
        cls.msg.left = Vector3(x=0.5, y=-0.2, z=1.0)
        cls.msg.right = Vector3(x=-0.8, y=0.3, z=0.7)

        cls.address = 17 # TODO: 1.

        cls.battery_num = 3 # TODO: 1.
        cls.motor_num = 3 # TODO: 1.
        cls.joint_num = 6 # TODO: 1.
        cls.end_effector_num = 3 # TODO: 1.

        cls.module_num = 3 # TODO: 1.

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.msgs = [None] * 16

    def tearDown(self):
        for sub in self.node.subscriptions:
            self.node.destroy_subscription(sub)

    def test_1_scaler_topics(self):
        self.node.create_subscription(
            EndEffector,
            "/end_effector",
            self.create_callback(0),
            10
        )
        self.node.create_subscription(
            Twist,
            "/cmd_vel",
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

        self.assertTrue(self.msgs[0] or self.msgs[1], "No topic found.")
        problems = []
        if not bool(self.msgs[0]): problems.append("Couldn't get message from '/end_effector' topic.")
        if not bool(self.msgs[1]): problems.append("Couldn't get message from '/cmd_vel' topic.")
        self.assertTrue(len(problems) == 0, f"\n{''.join(problems)}")

    def test_2_agevar_topics(self):
        time.sleep(2) # Wait until launch ends
        topic_list = [t[0] for t in self.node.get_topic_names_and_types()] # TODO: 1.
        idx = 0
        while 1:
            topic = f'/reseq/module{self.address + idx}/motor/setpoint'
            if topic not in topic_list:
                break
            self.node.create_subscription(
                Motors,
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

        self.assertTrue(self.msgs[:idx], "No topic found.")
        problems = []
        for i in range(idx):
            if not bool(self.msgs[i]): problems.append(f"Couldn't get message from '/reseq/module{self.address + i}/motor/setpoint' topic.")
        self.assertTrue(len(problems) == 0, f"\n{''.join(problems)}")

    def test_3_enea_topics(self):
        topic_list = [t[0] for t in self.node.get_topic_names_and_types()] # TODO: 1.
        idx = 0
        for vel in ["pitch", "head_pitch", "head_roll"]:
            n = 0
            while 1:
                topic = f'/reseq/module{self.address + n}/end_effector/{vel}/setpoint'
                if topic not in topic_list:
                    break
                self.node.create_subscription(
                    Int32,
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

        self.assertTrue(self.msgs[:idx], "No topic found.")
        problems = []
        for i in range(idx):
            if not bool(self.msgs[i]): problems.append(f"Couldn't get message from '/reseq/module{self.address + (i % (idx//3))}/end_effector/{['pitch', 'head_pitch', 'head_roll'][i // (idx//3)]}/setpoint' topic.")
        self.assertTrue(len(problems) == 0, f"\n{''.join(problems)}")

    def test_4_communication_topics(self):
        topic_names, topic_types = map(list, zip(*self.node.get_topic_names_and_types())) # TODO: 1.
        idx = 0
        n_motor = -1
        n_joint = -1
        n_end_effector = -1
        for st in ['motor', 'joint', 'end_effector']: # TODO: Add battery
            while 1:
                if st == 'motor':
                    topic = f'/reseq/module{self.address + idx}/{st}/feedback'
                    topic_type = Motors
                    n_motor += 1
                elif st == 'joint':
                    topic = f"/reseq/module{self.address+1 + ((idx-n_motor)//3)}/{st}/{['pitch', 'roll', 'yaw'][(idx-n_motor)%3]}/feedback"
                    topic_type = Float32
                    n_joint += 1
                else:
                    topic = f"/reseq/module{self.address + ((idx-n_motor-n_joint)//3)}/{st}/{['pitch', 'head_pitch', 'head_roll'][(idx-n_motor-n_joint)%3]}/feedback"
                    topic_type = Int32
                    n_end_effector += 1
                if topic not in topic_names:
                    break
                # Test topic type
                arg = topic_names.index(topic)
                expected_type_str = f"{topic_type.__module__.split('._')[0]}._{topic_type.__name__}"
                topic_type_str = topic_types[arg][0].replace('msg/', 'msg._').replace('/', '.')
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
            self.pub_.publish(self.msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if None not in self.msgs[:idx]:
                break
        
        self.assertTrue(self.msgs[:idx], "No topic found.")
        problems = []
        if n_motor != self.motor_num: problems.append(f"Number of motors doesn't match. Expected {self.motor_num}, got {n_motor}.")
        if n_joint != self.joint_num: problems.append(f"Number of joints doesn't match. Expected {self.joint_num}, got {n_joint}.")
        if n_end_effector != self.end_effector_num: problems.append(f"Number of end effectors doesn't match. Expected {self.end_effector_num}, got {n_end_effector}.")
        for i in range(idx):
            if n_motor > i: 
                if not bool(self.msgs[i]): problems.append(f"Couldn't get message from '/reseq/module{self.address + i}/motor/feedback' topic." + "\n")
            elif n_motor + n_joint > i:
                if not bool(self.msgs[i]): problems.append(f"Couldn't get message from '/reseq/module{self.address+1 + ((i-n_motor)//3)}/joint/{['pitch', 'roll', 'yaw'][(i-n_motor)%3]}/feedback' topic." + "\n")
            else:
                if not bool(self.msgs[i]): problems.append(f"Couldn't get message from '/reseq/module{self.address}/end_effector/{['pitch', 'head_pitch', 'head_roll'][(i-n_motor-n_joint)%3]}/feedback' topic." + "\n")
        self.assertTrue(len(problems) == 0, f"\n{''.join(problems)}")

    def test_5_jointpublisher_topics(self):
        self.node.create_subscription(
            JointState,
            "/joint_states",
            self.create_callback(0),
            10
        )
        idx = self.module_num + 1
        for i in range(1, idx):
            self.node.create_subscription(
                TwistStamped,
                f"/diff_controller{i}/cmd_vel",
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

        self.assertTrue(self.msgs[:idx], "No topic found.")
        problems = []
        if not bool(self.msgs[0]): problems.append("Couldn't get message from '/joint_states' topic.")
        for i in range(1, idx):
            if not bool(self.msgs[i]): problems.append(f"Couldn't get message from '/diff_controller{i}/cmd_vel' topic.")
        self.assertTrue(len(problems) == 0, f"\n{''.join(problems)}")

    def create_callback(self, index):
        def callback(msg):
            self.msgs[index] = msg
        return callback

def generate_test_description():
    # TODO: modify the config_file parameter.
    return LaunchDescription([
        ExecuteProcess(
            cmd=["ros2", "launch", "reseq_ros2", "reseq_launch.py", "config_file:=reseq_mk1_vcan.yaml"],
        ),

        # Start tests right away - no need to wait for anything in this example.
        # In a more complicated launch description, we might want this action happen
        # once some process starts or once some other event happens
        launch_testing.actions.ReadyToTest()
    ]), {
        'TestNodes': TestNodes,
    }

if __name__ == '__main__':
    unittest.main()
