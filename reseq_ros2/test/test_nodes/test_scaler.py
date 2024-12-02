import unittest
from time import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_testing.actions import ReadyToTest
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from reseq_interfaces.msg import EndEffector, Remote
from reseq_ros2.scaler import Scaler

# Launch the Scaler using the reseq MK1 vcan configurations
config_file = 'reseq_mk1_vcan.yaml'

@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'reseq_ros2', 'reseq_launch.py', f'config_file:={config_file}',
            ],
        ),
        ReadyToTest()
    ])

# Test

class TestScalerNode(unittest.TestCase):
    """Test class for the Scaler node's functionality."""

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)
        cls.executor = SingleThreadedExecutor()
        cls.context = rclpy.get_default_context()
        cls.node = rclpy.create_node('test_node', context=cls.context)
        cls.executor.add_node(cls.node)

        # Expected parameter values from scaler_consts.yaml
        cls.expected_params = {
            'r_linear_vel': [-0.27017, 0.27017],
            'r_inverse_radius': [-1.5385, 1.5385],
            'r_pitch_vel': [-183, 183],
            'r_head_pitch_vel': [-400, 400],
            'r_head_roll_vel': [-455, 455]
        }

        # Wait for the Scaler node to be up and running
        timeout_sec = 10
        scaler_node_found = False
        start_time = time()
        while time() - start_time < timeout_sec and not scaler_node_found:
            node_names = cls.node.get_node_names()
            scaler_node_found = 'scaler' in node_names
            if not scaler_node_found:
                rclpy.spin_once(cls.node, timeout_sec=1)

        if not scaler_node_found:
            raise RuntimeError('Scaler node was not found running within the timeout period.')

        # Create a client to get parameters of the scaler node
        cls.parameter_client = cls.node.create_client(GetParameters, '/scaler/get_parameters')

        if not cls.parameter_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError('Scaler node parameter service not available within the timeout period.')
        
        # Create a client to list all parameters of the scaler node
        cls.list_parameters_client = cls.node.create_client(ListParameters, '/scaler/list_parameters')

        if not cls.list_parameters_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError('Scaler node list parameters service not available within the timeout period.')
        
        # Initialize the scaler node
        cls.scaler = Scaler()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        cls.scaler.destroy_node()
        rclpy.shutdown()

    def get_parameter(self, node_name, parameter_name):
        request = GetParameters.Request()
        request.names = [parameter_name]
        future = self.parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        if response:
            return self.extract_value(response.values[0])
        else:
            self.fail(f"Failed to get parameter {parameter_name} from node {node_name}")
        
    def list_all_parameters(self, node_name):
        request = ListParameters.Request()
        future = self.list_parameters_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        if response:
            return response.result.names
        else:
            self.fail(f"Failed to list parameters from node {node_name}")

    def extract_value(self, parameter_value):
        if parameter_value.type == 1:
            return parameter_value.bool_value
        elif parameter_value.type == 2:
            return parameter_value.integer_value
        elif parameter_value.type == 3:
            return parameter_value.double_value
        elif parameter_value.type == 4:
            return parameter_value.string_value
        elif parameter_value.type == 5:
            return list(parameter_value.byte_array_value)
        elif parameter_value.type == 6:
            return list(parameter_value.bool_array_value)
        elif parameter_value.type == 7:
            return list(parameter_value.integer_array_value)
        elif parameter_value.type == 8:
            return list(parameter_value.double_array_value)
        elif parameter_value.type == 9:
            return list(parameter_value.string_array_value)
        return None

    def test_1_parameters(self):
        """Test that parameters are correctly set and check for unexpected or missing parameters."""
        all_parameters = self.list_all_parameters('scaler')
        unexpected_parameters = set(all_parameters[1:]) - set(self.expected_params.keys())
        missing_parameters = set(self.expected_params.keys()) - set(all_parameters[1:])

        self.assertEqual(len(unexpected_parameters), 0, f"Unexpected parameters found: {unexpected_parameters}")
        self.assertEqual(len(missing_parameters), 0, f"Missing parameters found: {missing_parameters}")

        for param_name, expected_value in self.expected_params.items():
            param_value = self.get_parameter('scaler', param_name)
            self.assertEqual(param_value, expected_value, f"{param_name} parameter value mismatch")

    def test_2_scale(self):
        """Test the scale function."""
        node = self.scaler
        
        # Test case: Scale linear value
        val = 0.5
        range = self.expected_params['r_linear_vel']
        expected_val = (val + 1) / 2 * (range[1] - range[0]) + range[0]
        scaled_val = node.scale(val, range)
        self.assertAlmostEqual(scaled_val, expected_val, places=5)

        # Test case: Scale inverse radius
        val = -0.5
        range = self.expected_params['r_inverse_radius']
        expected_val = (val + 1) / 2 * (range[1] - range[0]) + range[0]
        scaled_val = node.scale(val, range)
        self.assertAlmostEqual(scaled_val, expected_val, places=5)

    def test_3_agevarScaler(self):
        """Test the agevarScaler function."""
        node = self.scaler

        # Set initial constraints
        node.r_linear_vel = self.expected_params['r_linear_vel']
        node.r_inverse_radius = self.expected_params['r_inverse_radius']

        twist_msg = Twist()
        twist_msg.linear.x = 0.5
        twist_msg.angular.z = -0.5

        # Calculate expected values
        expected_linear_x = (twist_msg.linear.x + 1) / 2 * (node.r_linear_vel[1] - node.r_linear_vel[0]) + node.r_linear_vel[0]
        expected_angular_z = (twist_msg.angular.z + 1) / 2 * (node.r_inverse_radius[1] - node.r_inverse_radius[0]) + node.r_inverse_radius[0]
        expected_angular_z *= expected_linear_x

        # Call agevarScaler
        scaled_twist = node.agevarScaler(twist_msg)

        # Verify scaled values
        self.assertAlmostEqual(scaled_twist.linear.x, expected_linear_x, places=5)
        self.assertAlmostEqual(scaled_twist.angular.z, expected_angular_z, places=5)

    def test_4_endEffectorScaler(self):
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
        expected_pitch_vel = (end_effector_msg.pitch_vel + 1) / 2 * (node.r_pitch_vel[1] - node.r_pitch_vel[0]) + node.r_pitch_vel[0]
        expected_head_pitch_vel = (end_effector_msg.head_pitch_vel + 1) / 2 * (node.r_head_pitch_vel[1] - node.r_head_pitch_vel[0]) + node.r_head_pitch_vel[0]
        expected_head_roll_vel = (end_effector_msg.head_roll_vel + 1) / 2 * (node.r_head_roll_vel[1] - node.r_head_roll_vel[0]) + node.r_head_roll_vel[0]

        # Call endEffectorScaler
        scaled_end_effector = node.endEffectorScaler(end_effector_msg)

        # Verify scaled values
        self.assertAlmostEqual(scaled_end_effector.pitch_vel, expected_pitch_vel, places=5)
        self.assertAlmostEqual(scaled_end_effector.head_pitch_vel, expected_head_pitch_vel, places=5)
        self.assertAlmostEqual(scaled_end_effector.head_roll_vel, expected_head_roll_vel, places=5)

    def test_5_remote_callback(self):
        """Test the remote_callback function."""
        node = self.scaler

        # Set initial state
        node.set_parameters([
            Parameter('r_linear_vel', rclpy.Parameter.Type.DOUBLE_ARRAY, self.expected_params['r_linear_vel']),
            Parameter('r_inverse_radius', rclpy.Parameter.Type.DOUBLE_ARRAY, self.expected_params['r_inverse_radius']),
            Parameter('r_pitch_vel', rclpy.Parameter.Type.INTEGER_ARRAY, self.expected_params['r_pitch_vel']),
            Parameter('r_head_pitch_vel', rclpy.Parameter.Type.INTEGER_ARRAY, self.expected_params['r_head_pitch_vel']),
            Parameter('r_head_roll_vel', rclpy.Parameter.Type.INTEGER_ARRAY, self.expected_params['r_head_roll_vel']),
        ])

        remote_msg = Remote()
        remote_msg.right.y = 0.5
        remote_msg.right.x = -0.5
        remote_msg.left.y = 1.0
        remote_msg.left.z = 0.7
        remote_msg.left.x = -0.3

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
        expected_linear_x = (remote_msg.right.y + 1) / 2 * (node.r_linear_vel[1] - node.r_linear_vel[0]) + node.r_linear_vel[0]
        expected_angular_z = (- remote_msg.right.x + 1) / 2 * (node.r_inverse_radius[1] - node.r_inverse_radius[0]) + node.r_inverse_radius[0]
        expected_angular_z *= expected_linear_x

        # Calculate expected endeffector values
        expected_pitch_vel = (remote_msg.left.y + 1) / 2 * (node.r_pitch_vel[1] - node.r_pitch_vel[0]) + node.r_pitch_vel[0]
        expected_head_pitch_vel = (remote_msg.left.z + 1) / 2 * (node.r_head_pitch_vel[1] - node.r_head_pitch_vel[0]) + node.r_head_pitch_vel[0]
        expected_head_roll_vel = (remote_msg.left.x + 1) / 2 * (node.r_head_roll_vel[1] - node.r_head_roll_vel[0]) + node.r_head_roll_vel[0]
        
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
        self.assertAlmostEqual(self.end_effector_msg.head_pitch_vel, expected_head_pitch_vel, places=5)
        self.assertAlmostEqual(self.end_effector_msg.head_roll_vel, expected_head_roll_vel, places=5)


if __name__ == '__main__':
    unittest.main()