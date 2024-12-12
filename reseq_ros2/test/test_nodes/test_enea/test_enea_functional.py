import unittest
from time import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import Int32

from reseq_interfaces.msg import EndEffector
from reseq_ros2.enea import EE_Enum, Enea

# Function Test


class TestEneaFunctional(unittest.TestCase):
    """Test class for the Enea node's functionality."""

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)
        cls.executor = SingleThreadedExecutor()
        cls.enea = Enea()
        cls.executor.add_node(cls.enea)

        # Expected parameter values from enea_consts.yaml
        cls.expected_params = {
            'pitch': 567,
            'head_pitch': 663,
            'head_roll': 512,
            'servo_speed': 200,
            'r_pitch': [200, 750],
            'r_head_pitch': [0, 800],
            'r_head_roll': [0, 1023],
            'pitch_conv': 0.75,
            'end_effector': 0x11,  # received from reseq_mk1_vcan.yaml
        }

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.enea.destroy_node()
        rclpy.shutdown()

    def test_4_constrain_delta_pitch(self):
        """Test the constrain_delta_pitch function."""
        node = self.enea

        # Set initial pitch and constraints
        node.pitch = 500
        node.r_pitch = self.expected_params['r_pitch']

        # Test case: dp within constraints
        dp = 50
        result = node.constrain_delta_pitch(dp)
        self.assertEqual(result, 50)

        # Test case: dp exceeding upper constraint
        dp = 300
        result = node.constrain_delta_pitch(dp)
        self.assertEqual(result, 250)  # pitch + dp should not exceed 750

        # Test case: dp exceeding lower constraint
        dp = -400
        result = node.constrain_delta_pitch(dp)
        self.assertEqual(result, -300)  # pitch + dp should not go below 200

        # Test case: dp exactly at upper constraint
        dp = 250
        result = node.constrain_delta_pitch(dp)
        self.assertEqual(result, 250)

        # Test case: dp exactly at lower constraint
        dp = -300
        result = node.constrain_delta_pitch(dp)
        self.assertEqual(result, -300)

    def test_5_constrain(self):
        """Test the constrain function."""
        node = self.enea

        # Set initial constraints
        node.r_pitch = self.expected_params['r_pitch']
        node.r_head_pitch = self.expected_params['r_head_pitch']
        node.r_head_roll = self.expected_params['r_head_roll']

        # Test case: values above constraints
        node.pitch = 800
        node.head_pitch = 900
        node.head_roll = 1050

        # Apply constraints
        node.constrain()

        # Verify constrained values
        self.assertEqual(node.pitch, 750)
        self.assertEqual(node.head_pitch, 800)
        self.assertEqual(node.head_roll, 1023)

        # Test case: values within constraints
        node.pitch = 500
        node.head_pitch = 600
        node.head_roll = 700

        # Apply constraints
        node.constrain()

        # Verify constrained values
        self.assertEqual(node.pitch, 500)
        self.assertEqual(node.head_pitch, 600)
        self.assertEqual(node.head_roll, 700)

        # Test case: values below constraints
        node.pitch = 100
        node.head_pitch = -50
        node.head_roll = -50

        # Apply constraints
        node.constrain()

        # Verify constrained values
        self.assertEqual(node.pitch, 200)
        self.assertEqual(node.head_pitch, 0)
        self.assertEqual(node.head_roll, 0)

    def test_6_post(self):
        """Test the post function."""
        node = self.enea

        # Mock values to publish
        node.pitch = 567
        node.head_pitch = 663
        node.head_roll = 512

        # Create publishers to capture messages
        node.pubs = {
            EE_Enum.PITCH: node.create_publisher(Int32, 'test_pitch', 10),
            EE_Enum.HEAD_PITCH: node.create_publisher(Int32, 'test_head_pitch', 10),
            EE_Enum.HEAD_ROLL: node.create_publisher(Int32, 'test_head_roll', 10),
        }

        # Capture published messages
        pitch_msg = None
        head_pitch_msg = None
        head_roll_msg = None

        def pitch_callback(msg):
            nonlocal pitch_msg
            pitch_msg = msg

        def head_pitch_callback(msg):
            nonlocal head_pitch_msg
            head_pitch_msg = msg

        def head_roll_callback(msg):
            nonlocal head_roll_msg
            head_roll_msg = msg

        node.create_subscription(Int32, 'test_pitch', pitch_callback, 10)
        node.create_subscription(Int32, 'test_head_pitch', head_pitch_callback, 10)
        node.create_subscription(Int32, 'test_head_roll', head_roll_callback, 10)

        # Call the post function
        node.post()

        # Spin the node to process the publishing and subscription
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
            if pitch_msg is not None and head_pitch_msg is not None and head_roll_msg is not None:
                break

        # Verify published messages
        self.assertIsNotNone(pitch_msg)
        self.assertEqual(pitch_msg.data, 567)
        self.assertIsNotNone(head_pitch_msg)
        self.assertEqual(head_pitch_msg.data, 663)
        self.assertIsNotNone(head_roll_msg)
        self.assertEqual(head_roll_msg.data, 512)

    def test_7_consume_velocities(self):
        """Test the consume_velocities function."""
        node = self.enea

        # Set initial state
        node.set_parameters(
            [
                Parameter('pitch', rclpy.Parameter.Type.INTEGER, self.expected_params['pitch']),
                Parameter(
                    'head_pitch', rclpy.Parameter.Type.INTEGER, self.expected_params['head_pitch']
                ),
                Parameter(
                    'head_roll', rclpy.Parameter.Type.INTEGER, self.expected_params['head_roll']
                ),
                Parameter(
                    'servo_speed',
                    rclpy.Parameter.Type.INTEGER,
                    self.expected_params['servo_speed'],
                ),
                Parameter(
                    'r_pitch', rclpy.Parameter.Type.INTEGER_ARRAY, self.expected_params['r_pitch']
                ),
                Parameter(
                    'r_head_pitch',
                    rclpy.Parameter.Type.INTEGER_ARRAY,
                    self.expected_params['r_head_pitch'],
                ),
                Parameter(
                    'r_head_roll',
                    rclpy.Parameter.Type.INTEGER_ARRAY,
                    self.expected_params['r_head_roll'],
                ),
                Parameter(
                    'pitch_conv', rclpy.Parameter.Type.DOUBLE, self.expected_params['pitch_conv']
                ),
                Parameter(
                    'end_effector',
                    rclpy.Parameter.Type.INTEGER,
                    self.expected_params['end_effector'],
                ),
            ]
        )

        end_effector_msg = EndEffector()
        end_effector_msg.pitch_vel = 0.1
        end_effector_msg.head_pitch_vel = 0.2
        end_effector_msg.head_roll_vel = 0.3

        # Simulate time passage
        initial_time = -1
        current_time = time()
        dt = current_time - initial_time

        # Calculate the expected state
        dp = -end_effector_msg.pitch_vel * dt
        dp = node.constrain_delta_pitch(dp)
        expected_pitch = node.pitch + dp
        expected_head_pitch = (
            node.head_pitch + end_effector_msg.head_pitch_vel * dt + node.pitch_conv * dp
        )
        expected_head_roll = node.head_roll - end_effector_msg.head_roll_vel * dt

        # Constrain values
        expected_pitch = max(min(expected_pitch, node.r_pitch[1]), node.r_pitch[0])
        expected_head_pitch = max(
            min(expected_head_pitch, node.r_head_pitch[1]), node.r_head_pitch[0]
        )
        expected_head_roll = max(min(expected_head_roll, node.r_head_roll[1]), node.r_head_roll[0])

        # Call consume_velocities
        node.consume_velocities(end_effector_msg)

        # Verify updated state
        self.assertAlmostEqual(node.pitch, expected_pitch, places=2)
        self.assertAlmostEqual(node.head_pitch, expected_head_pitch, places=2)
        self.assertAlmostEqual(node.head_roll, expected_head_roll, places=2)


if __name__ == '__main__':
    unittest.main()
