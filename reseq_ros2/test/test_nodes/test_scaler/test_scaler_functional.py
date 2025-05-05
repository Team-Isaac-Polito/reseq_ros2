import unittest

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from reseq_interfaces.msg import EndEffector, Remote
from reseq_ros2.scaler import Scaler

# Function Test


class TestScalerFunctional(unittest.TestCase):
    """Test class for the Scaler node's functionality."""

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)
        cls.executor = SingleThreadedExecutor()
        cls.scaler = Scaler()
        cls.executor.add_node(cls.scaler)

        # Expected parameter values from scaler_consts.yaml
        cls.expected_params = {
            'version': 'mk1',
            'r_linear_vel': [-0.27017, 0.27017],
            'r_inverse_radius': [-1.5385, 1.5385],
            'r_angular_vel': [-2.8387, 2.8387],
            'r_pitch_vel': [-183, 183],
            'r_head_pitch_vel': [-400, 400],
            'r_head_roll_vel': [-455, 455],
        }

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.scaler.destroy_node()
        rclpy.shutdown()

    def test_4_scale(self):
        """Test the scale function."""
        node = self.scaler

        # Test case: Scale linear value
        val = 0.5
        scaling_range = self.expected_params['r_linear_vel']
        expected_val = (val + 1) / 2 * (scaling_range[1] - scaling_range[0]) + scaling_range[0]
        scaled_val = node.scale(val, scaling_range)
        self.assertAlmostEqual(scaled_val, expected_val, places=5)

        # Test case: Scale inverse radius
        val = -0.5
        scaling_range = self.expected_params['r_inverse_radius']
        expected_val = (val + 1) / 2 * (scaling_range[1] - scaling_range[0]) + scaling_range[0]
        scaled_val = node.scale(val, scaling_range)
        self.assertAlmostEqual(scaled_val, expected_val, places=5)

    def test_5_agevarScaler(self):
        """Test the agevarScaler function."""
        node = self.scaler

        # Set initial constraints
        node.r_linear_vel = self.expected_params['r_linear_vel']
        node.r_inverse_radius = self.expected_params['r_inverse_radius']

        twist_msg = Twist()
        twist_msg.linear.x = 0.5
        twist_msg.angular.z = -0.5

        # Calculate expected values
        expected_linear_x = (twist_msg.linear.x + 1) / 2 * (
            node.r_linear_vel[1] - node.r_linear_vel[0]
        ) + node.r_linear_vel[0]
        expected_angular_z = (twist_msg.angular.z + 1) / 2 * (
            node.r_inverse_radius[1] - node.r_inverse_radius[0]
        ) + node.r_inverse_radius[0]
        expected_angular_z *= expected_linear_x

        # Call agevarScaler
        scaled_twist = node.agevarScaler(twist_msg)

        # Verify scaled values
        self.assertAlmostEqual(scaled_twist.linear.x, expected_linear_x, places=5)
        self.assertAlmostEqual(scaled_twist.angular.z, expected_angular_z, places=5)

    def test_6_endEffectorScaler(self):
        """Test the endEffectorScaler function."""
        node = self.scaler

        # Set initial constraints
        node.r_pitch_vel = self.expected_params['r_pitch_vel']
        node.r_head_pitch_vel = self.expected_params['r_head_pitch_vel']
        node.r_head_roll_vel = self.expected_params['r_head_roll_vel']

        end_effector_msg = EndEffector()
        end_effector_msg.pitch_vel = 1.0
        end_effector_msg.head_pitch_vel = 0.7
        end_effector_msg.head_roll_vel = -0.3

        # Calculate expected values
        expected_pitch_vel = (end_effector_msg.pitch_vel + 1) / 2 * (
            node.r_pitch_vel[1] - node.r_pitch_vel[0]
        ) + node.r_pitch_vel[0]
        expected_head_pitch_vel = (end_effector_msg.head_pitch_vel + 1) / 2 * (
            node.r_head_pitch_vel[1] - node.r_head_pitch_vel[0]
        ) + node.r_head_pitch_vel[0]
        expected_head_roll_vel = (end_effector_msg.head_roll_vel + 1) / 2 * (
            node.r_head_roll_vel[1] - node.r_head_roll_vel[0]
        ) + node.r_head_roll_vel[0]

        # Call endEffectorScaler
        scaled_end_effector = node.endEffectorScaler(end_effector_msg)

        # Verify scaled values
        self.assertAlmostEqual(scaled_end_effector.pitch_vel, expected_pitch_vel, places=5)
        self.assertAlmostEqual(
            scaled_end_effector.head_pitch_vel, expected_head_pitch_vel, places=5
        )
        self.assertAlmostEqual(scaled_end_effector.head_roll_vel, expected_head_roll_vel, places=5)

    def test_7_remote_callback(self):
        """Test the remote_callback function."""
        node = self.scaler

        # Set initial state
        node.set_parameters(
            [
                Parameter(
                    'r_linear_vel',
                    rclpy.Parameter.Type.DOUBLE_ARRAY,
                    self.expected_params['r_linear_vel'],
                ),
                Parameter(
                    'r_inverse_radius',
                    rclpy.Parameter.Type.DOUBLE_ARRAY,
                    self.expected_params['r_inverse_radius'],
                ),
                Parameter(
                    'r_pitch_vel',
                    rclpy.Parameter.Type.INTEGER_ARRAY,
                    self.expected_params['r_pitch_vel'],
                ),
                Parameter(
                    'r_head_pitch_vel',
                    rclpy.Parameter.Type.INTEGER_ARRAY,
                    self.expected_params['r_head_pitch_vel'],
                ),
                Parameter(
                    'r_head_roll_vel',
                    rclpy.Parameter.Type.INTEGER_ARRAY,
                    self.expected_params['r_head_roll_vel'],
                ),
                Parameter(
                    'version',
                    rclpy.Parameter.Type.STRING,
                    self.expected_params['version'],
                ),
                Parameter(
                    'r_angular_vel',
                    rclpy.Parameter.Type.DOUBLE_ARRAY,
                    self.expected_params['r_angular_vel'],
                ),
            ]
        )

        remote_msg = Remote()
        remote_msg.right.y = 0.5
        remote_msg.right.x = -0.5
        remote_msg.left.y = 1.0
        remote_msg.left.z = 0.7
        remote_msg.left.x = -0.3
        remote_msg.buttons = [False, False, False, False, False, True, True, True, True, True]

        # Capture published messages
        self.cmd_vel_msg = None
        self.end_effector_msg = None

        def cmd_vel_callback(msg):
            self.cmd_vel_msg = msg

        def end_effector_callback(msg):
            self.end_effector_msg = msg

        node.create_subscription(Twist, '/cmd_vel', cmd_vel_callback, 10)
        node.create_subscription(EndEffector, '/end_effector', end_effector_callback, 10)

        # Calculate expected agevar values
        expected_linear_x = (remote_msg.right.y + 1) / 2 * (
            node.r_linear_vel[1] - node.r_linear_vel[0]
        ) + node.r_linear_vel[0]
        expected_angular_z = (-remote_msg.right.x + 1) / 2 * (
            node.r_inverse_radius[1] - node.r_inverse_radius[0]
        ) + node.r_inverse_radius[0]
        expected_angular_z *= expected_linear_x

        # Calculate expected endeffector values
        expected_pitch_vel = (remote_msg.left.y + 1) / 2 * (
            node.r_pitch_vel[1] - node.r_pitch_vel[0]
        ) + node.r_pitch_vel[0]
        expected_head_pitch_vel = (remote_msg.left.z + 1) / 2 * (
            node.r_head_pitch_vel[1] - node.r_head_pitch_vel[0]
        ) + node.r_head_pitch_vel[0]
        expected_head_roll_vel = (remote_msg.left.x + 1) / 2 * (
            node.r_head_roll_vel[1] - node.r_head_roll_vel[0]
        ) + node.r_head_roll_vel[0]

        # Call the remote_callback
        node.remote_callback(remote_msg)

        # Spin the node to process the publishing and subscription
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
            if self.cmd_vel_msg is not None and self.end_effector_msg is not None:
                break

        # Verify cmd_vel message
        self.assertIsNotNone(self.cmd_vel_msg)
        self.assertAlmostEqual(self.cmd_vel_msg.linear.x, expected_linear_x, places=5)
        self.assertAlmostEqual(self.cmd_vel_msg.angular.z, expected_angular_z, places=5)

        # Verify end_effector message
        self.assertIsNotNone(self.end_effector_msg)
        self.assertAlmostEqual(self.end_effector_msg.pitch_vel, expected_pitch_vel, places=5)
        self.assertAlmostEqual(
            self.end_effector_msg.head_pitch_vel, expected_head_pitch_vel, places=5
        )
        self.assertAlmostEqual(
            self.end_effector_msg.head_roll_vel, expected_head_roll_vel, places=5
        )


if __name__ == '__main__':
    unittest.main()
