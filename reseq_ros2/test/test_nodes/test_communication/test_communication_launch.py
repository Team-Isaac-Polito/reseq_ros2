from time import time
import unittest

import rclpy
import pytest
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_testing.actions import ReadyToTest

from rcl_interfaces.srv import GetParameters, ListParameters

# Launch the Communication using the reseq MK1 vcan configurations
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

# Launch Test

class TestCommunicationLaunch(unittest.TestCase):
    """Test class for verifying that the Communication node launches correctly with the expected parameters."""

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)
        cls.node = rclpy.create_node('test_node')

        # Expected parameter values
        cls.expected_params = {
            'can_channel': 'vcan0',
            'modules': [0x11, 0x12, 0x13],  # received from reseq_mk1_vcan.yaml
            'joints': [0x12, 0x13],   # received from reseq_mk1_vcan.yaml
            'end_effector': 0x11  # received from reseq_mk1_vcan.yaml
        }
        cls.timeout_sec = 10

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

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

    def test_1_nodes_up(self):
        """Test that the Communication node is up and running."""
        expected_node = 'communication'

        # Wait for the Communication node to be up and running
        communication_node_found = False
        start_time = time()
        while time() - start_time < self.timeout_sec and not communication_node_found:
            node_names = self.node.get_node_names()
            communication_node_found = expected_node in node_names
            if not communication_node_found:
                rclpy.spin_once(self.node, timeout_sec=1)

        # Validate the state of the node
        self.assertTrue(communication_node_found, 'Communication node was not found running within the timeout period.')

    def test_2_parameters_set(self):
        """Test that parameters are correctly set on the Communication node."""
        # Create a client to get parameters of the communication node
        parameter_client = self.node.create_client(GetParameters, '/communication/get_parameters')
        client_found = parameter_client.wait_for_service(timeout_sec=self.timeout_sec)
        self.assertTrue(client_found, 'Communication node parameter service not available within the timeout period.')

        for param_name, expected_value in self.expected_params.items():
            request = GetParameters.Request()
            request.names = [param_name]
            future = parameter_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            response = future.result()

            # Verify the response
            self.assertIsNotNone(response, f'Failed to get parameter {param_name} from Communication node')
            
            param_value = response.values[0]
            self.assertEqual(self.extract_value(param_value), expected_value, f'{param_name} parameter value mismatch')

    def test_3_missing_parameters(self):
        """Test for unexpected or missing parameters on the Communication node."""
        # Create a client to list all parameters of the communication node
        list_parameters_client = self.node.create_client(ListParameters, '/communication/list_parameters')
        client_found = list_parameters_client.wait_for_service(timeout_sec=self.timeout_sec)
        self.assertTrue(client_found, 'Communication node list parameters service not available within the timeout period.')

        request = ListParameters.Request()
        future = list_parameters_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        # Verify the response
        self.assertIsNotNone(response, 'Failed to list parameters from Communication node')

        all_parameters = response.result.names[1:]  # all params except the 'use_sim_time' parameter
        unexpected_parameters = set(all_parameters) - set(self.expected_params.keys())
        missing_parameters = set(self.expected_params.keys()) - set(all_parameters)

        self.assertEqual(len(unexpected_parameters), 0, f'Unexpected parameters found: {unexpected_parameters}')
        self.assertEqual(len(missing_parameters), 0, f'Missing parameters found: {missing_parameters}')


if __name__ == '__main__':
    unittest.main()
