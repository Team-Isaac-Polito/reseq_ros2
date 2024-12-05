import unittest
from math import pi
import rclpy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32, Float32
from reseq_interfaces.msg import Motors
import reseq_ros2.constants as rc
from rclpy.executors import SingleThreadedExecutor
from reseq_ros2.joint_publisher import JointPublisher, State


# Function Test

class TestJointPublisherFunctional(unittest.TestCase):
    """Test class for the JointPublisher node's functionality."""

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)
        cls.executor = SingleThreadedExecutor()
        cls.joint_publisher = JointPublisher()
        cls.executor.add_node(cls.joint_publisher)

        # Expected parameter values from joint_pub_consts.yaml
        cls.expected_params = {
            'modules': [0x11, 0x12, 0x13],  # received from reseq_mk1_vcan.yaml
            'joints': [0x12, 0x13],   # received from reseq_mk1_vcan.yaml
            'end_effector': 0x11,  # received from reseq_mk1_vcan.yaml
            'b': 0.18395,  # received from agevar_consts_mk1.yaml
            'arm_pitch_origin': 200,
            'head_pitch_origin': 800,
            'head_roll_origin': 512,
            'arm_pitch_gain': 0.50943,
            'vel_gain': 0.52522
        }

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.joint_publisher.destroy_node()
        rclpy.shutdown()
    
    def test_4_init_state(self):
        """Test the init_state function."""
        node = self.joint_publisher
        
        # Call init_state
        node.init_state(self.expected_params['modules'], self.expected_params['joints'], self.expected_params['end_effector'])

        # Verify controller publishers
        controller_pubs = node.controller_pubs[1:]
        self.assertEqual(len(controller_pubs), len(self.expected_params['modules']))
        for i, pub in enumerate(controller_pubs):
            self.assertEqual(pub.topic, f'/diff_controller{i+0x11 % 16}/cmd_vel')

        # Verify wheel velocities
        wheel_velocities = node.wheel_velocities[1:]
        self.assertEqual(len(wheel_velocities), len(self.expected_params['modules']))
        for vel in wheel_velocities:
            self.assertEqual(vel, [0.0, 0.0])

        # Verify position states for joints and end effector
        for mod in self.expected_params['modules']:
            id = mod % 16
            if mod in self.expected_params['joints']:
                for x in node.states_from_type(rc.StateType.JOINT_FEEDBACK):
                    self.assertIn((mod, x), node.position_states)
                    self.assertEqual(node.position_states[(mod, x)].name, x + f"_{id}_joint")
                    self.assertEqual(node.position_states[(mod, x)].value, 0.0)

            if mod == self.expected_params['end_effector']:
                for x in node.states_from_type(rc.StateType.END_EFFECTOR_FEEDBACK):
                    self.assertIn((mod, x), node.position_states)
                    self.assertEqual(node.position_states[(mod, x)].name, x + f"_{id}_joint")
                    self.assertEqual(node.position_states[(mod, x)].value, 0.0)

    def test_5_create_subs(self):
        """Test the create_subs function."""
        node = self.joint_publisher

        # Call the create_subs method
        node.create_subs(self.expected_params['modules'], self.expected_params['joints'], self.expected_params['end_effector'])

        # Verify motor feedback subscriptions
        for mod in self.expected_params['modules']:
            for topic in node.topics_from_type(rc.StateType.MOTOR_FEEDBACK):
                sub_topic = f"/reseq/module{mod}/{topic}"
                self.assertTrue(any(sub.topic_name == sub_topic for sub in node.subscriptions), f"Subscription not found for {sub_topic}")

            # Verify joint feedback subscriptions
            if mod in self.expected_params['joints']:
                for topic in node.topics_from_type(rc.StateType.JOINT_FEEDBACK):
                    sub_topic = f"/reseq/module{mod}/{topic}"
                    self.assertTrue(any(sub.topic_name == sub_topic for sub in node.subscriptions), f"Subscription not found for {sub_topic}")

            # Verify end effector feedback subscriptions
            if mod == self.expected_params['end_effector']:
                for topic in node.topics_from_type(rc.StateType.END_EFFECTOR_FEEDBACK):
                    sub_topic = f"/reseq/module{mod}/{topic}"
                    self.assertTrue(any(sub.topic_name == sub_topic for sub in node.subscriptions), f"Subscription not found for {sub_topic}")

    def test_6_motor_feedback(self):
        """Test the motor feedback."""
        node = self.joint_publisher

        # Set initial gain
        node.velocity_gain = self.expected_params['vel_gain']

        motors_msg = Motors()
        motors_msg.left = 10.0
        motors_msg.right = 20.0

        # Call update callback
        node.update_callback(motors_msg, 0x11, 'motor/feedback', rc.StateType.MOTOR_FEEDBACK)

        # Calculate expected values
        expected_left = motors_msg.left * node.velocity_gain
        expected_right = motors_msg.right * node.velocity_gain

        # Verify wheel velocities
        self.assertAlmostEqual(node.wheel_velocities[0], [expected_left, expected_right], places=5)

    def test_7_joint_feedback(self):
        """Test the joint feedback."""
        node = self.joint_publisher

        # Test case: values within constraints
        angle_msg = Float32()
        angle_msg.data = 45.0

        # Call update callback
        node.update_callback(angle_msg, 0x12, 'joint/yaw/feedback', rc.StateType.JOINT_FEEDBACK)

        # Verify position states
        expected_angle = 45.0 * pi / 180.0
        self.assertAlmostEqual(node.position_states[(0x12, 'joint_y')].value, expected_angle, places=5)

        # Test case: values not within constraints
        angle_msg.data = 180.0

        # Call update callback
        node.update_callback(angle_msg, 0x12, 'joint/yaw/feedback', rc.StateType.JOINT_FEEDBACK)

        # Verify position states
        expected_angle = -pi
        self.assertAlmostEqual(node.position_states[(0x12, 'joint_y')].value, expected_angle, places=5)

    def test_8_end_effector_feedback(self):
        """Test the end effector feedback."""
        node = self.joint_publisher

        # Set initial values and parameters
        node.arm_pitch_origin = self.expected_params['arm_pitch_origin']
        node.head_pitch_origin = self.expected_params['head_pitch_origin']
        node.head_roll_origin = self.expected_params['head_roll_origin']
        node.arm_pitch_gain = self.expected_params['arm_pitch_gain']

        # Test case for end effector pitch feedback
        int32_msg = Int32(data=600)
        node.update_callback(int32_msg, 0x11, 'end_effector/pitch/feedback', rc.StateType.END_EFFECTOR_FEEDBACK)

        # Verify pitch position states
        expected_value_pitch = (int32_msg.data - self.expected_params['arm_pitch_origin']) * rc.lsb_to_rads * self.expected_params['arm_pitch_gain'] * (-1)
        self.assertAlmostEqual(node.position_states[(0x11, 'arm_pitch')].value, expected_value_pitch, places=5)

        # Test case for end effector head pitch feedback
        int32_msg = Int32(data=650)
        node.update_callback(int32_msg, 0x11, 'end_effector/head_pitch/feedback', rc.StateType.END_EFFECTOR_FEEDBACK)

        # Verify head pitch position states
        expected_value_head_pitch = (int32_msg.data - self.expected_params['head_pitch_origin']) * rc.lsb_to_rads
        self.assertAlmostEqual(node.position_states[(0x11, 'arm_head_pitch')].value, expected_value_head_pitch, places=5)

        # Test case for end effector head roll feedback
        int32_msg = Int32(data=700)
        node.update_callback(int32_msg, 0x11, 'end_effector/head_roll/feedback', rc.StateType.END_EFFECTOR_FEEDBACK)

        # Verify head roll position states
        expected_value_head_roll = (int32_msg.data - self.expected_params['head_roll_origin']) * rc.lsb_to_rads
        self.assertAlmostEqual(node.position_states[(0x11, 'arm_head_roll')].value, expected_value_head_roll, places=5)

    def test_9_broadcast_states(self):
        """Test the broadcast_states function."""
        node = self.joint_publisher

        # Set initial parameters
        node.b = self.expected_params['b']
        node.wheel_velocities = [[10.0, 20.0], [15.0, 25.0], [20.0, 30.0]]
        node.position_states = {
            (0x11, 'pitch'): State('pitch_joint', 0.5),
            (0x12, 'yaw'): State('yaw_joint', 0.7),
            (0x13, 'roll'): State('roll_joint', 0.9)
        }

        # Capture the JointState messages
        joint_states = None

        def joint_states_callback(msg):
            nonlocal joint_states
            joint_states = msg

        node.create_subscription(JointState, '/joint_states', joint_states_callback, 10)

        # Call broadcast_states
        node.broadcast_states()

        # Spin the node to process the publishing and subscription
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
            if joint_states is not None:
                break

        # Verify the JointState messages
        self.assertIsNotNone(joint_states)
        self.assertEqual(joint_states.name, ['pitch_joint', 'yaw_joint', 'roll_joint'])
        self.assertEqual(list(joint_states.position), [0.5, 0.7, 0.9])
        self.assertTrue(joint_states.header.stamp.sec > 0 or joint_states.header.stamp.nanosec > 0, "JointState timestamp is not set.")

        # Capture the TwistStamped messages
        twist_stamped_msgs = []

        def twist_stamped_callback(msg):
            twist_stamped_msgs.append(msg)

        for i in range(len(node.wheel_velocities)):
            node.create_subscription(TwistStamped, f'/diff_controller{i}/cmd_vel', twist_stamped_callback, 10)

        # Call broadcast_states again to test TwistStamped messages
        node.broadcast_states()

        # Spin the node to process the publishing and subscription
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(twist_stamped_msgs) >= len(node.wheel_velocities):
                break

        # Verify the TwistStamped messages
        self.assertEqual(len(twist_stamped_msgs), len(node.wheel_velocities))
        for i, vel in enumerate(node.wheel_velocities):
            expected_v = (vel[0] + vel[1]) / 2
            expected_w = (vel[1] - vel[0]) / node.b
            self.assertAlmostEqual(twist_stamped_msgs[i].twist.linear.x, expected_v, places=5)
            self.assertAlmostEqual(twist_stamped_msgs[i].twist.angular.z, expected_w, places=5)
            self.assertTrue(twist_stamped_msgs[i].header.stamp.sec > 0 or twist_stamped_msgs[i].header.stamp.nanosec > 0, "TwistStamped timestamp is not set.")


if __name__ == '__main__':
    unittest.main()
