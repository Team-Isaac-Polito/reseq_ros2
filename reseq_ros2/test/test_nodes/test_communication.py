import unittest
import pytest
import rclpy
from time import time
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.srv import GetParameters, ListParameters
import reseq_ros2.constants as rc
from rclpy.parameter import Parameter

from reseq_ros2.communication import Communication

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_testing.actions import ReadyToTest

# Test the enea using the reseq MK1 vcan configurations
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

class TestCommunicationNode(unittest.TestCase):
    """Test class for the Communication node's functionality."""

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)
        cls.executor = SingleThreadedExecutor()
        cls.context = rclpy.get_default_context()
        cls.node = rclpy.create_node('test_node', context=cls.context)
        cls.executor.add_node(cls.node)

        # Expected parameter values
        cls.expected_params = {
            'can_channel': 'vcan0',
            'modules': [0x11, 0x12, 0x13],  # received from reseq_mk1_vcan.yaml
            'joints': [0x12, 0x13],   # received from reseq_mk1_vcan.yaml
            'end_effector': 0x11  # received from reseq_mk1_vcan.yaml
        }

        # Wait for the Communication node to be up and running
        timeout_sec = 10
        communication_node_found = False
        start_time = time()
        while time() - start_time < timeout_sec and not communication_node_found:
            node_names = cls.node.get_node_names()
            communication_node_found = 'communication' in node_names
            if not communication_node_found:
                rclpy.spin_once(cls.node, timeout_sec=1)

        if not communication_node_found:
            raise RuntimeError('Communication node was not found running within the timeout period.')
        
        # Create a client to get parameters of the communication node
        cls.parameter_client = cls.node.create_client(GetParameters, '/communication/get_parameters')

        if not cls.parameter_client.wait_for_service(timeout_sec=10):
            raise RuntimeError('Communication node parameter service not available within the timeout period.')

        # Create a client to list all parameters of the communication node
        cls.list_parameters_client = cls.node.create_client(ListParameters, '/communication/list_parameters')

        if not cls.list_parameters_client.wait_for_service(timeout_sec=10):
            raise RuntimeError('Communication node list parameters service not available within the timeout period.')

        # Initialize the communication node
        cls.communication = Communication()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        cls.communication.destroy_node()
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
        all_parameters = self.list_all_parameters('communication')
        unexpected_parameters = set(all_parameters[1:]) - set(self.expected_params.keys())
        missing_parameters = set(self.expected_params.keys()) - set(all_parameters[1:])

        self.assertEqual(len(unexpected_parameters), 0, f"Unexpected parameters found: {unexpected_parameters}")
        self.assertEqual(len(missing_parameters), 0, f"Missing parameters found: {missing_parameters}")

        for param_name, expected_value in self.expected_params.items():
            param_value = self.get_parameter('communication', param_name)
            self.assertEqual(param_value, expected_value, f"{param_name} parameter value mismatch")

    def test_2_create_module_pubs(self):
        """Test the create_module_pubs function."""
        node = self.communication

        # Set initial parameters
        node.set_parameters([
            Parameter('modules', rclpy.Parameter.Type.INTEGER_ARRAY, self.expected_params['modules']),
            Parameter('joints', rclpy.Parameter.Type.INTEGER_ARRAY, self.expected_params['joints']),
            Parameter('end_effector', rclpy.Parameter.Type.INTEGER, self.expected_params['end_effector']),
        ])

        # Verify publishers for each module
        for mod in self.expected_params['modules']:
            has_joint = mod in self.expected_params['joints']
            has_end_effector = mod == self.expected_params['end_effector']
            pubs = node.create_module_pubs(mod, has_joint, has_end_effector)
            for topic in node.topics_from_direction(rc.Direction.IN):
                if topic.name.split("/")[0] == "joint" and not has_joint:
                    continue
                if topic.name.split("/")[0] == "end_effector" and not has_end_effector:
                    continue
                self.assertIn(topic.name, pubs, f"Publisher not found for {topic.name}")

    def test_3_create_module_subs(self):
        """Test the create_module_subs function."""
        node = self.communication

        # Set initial parameters
        node.set_parameters([
            Parameter('modules', rclpy.Parameter.Type.INTEGER_ARRAY, self.expected_params['modules']),
            Parameter('joints', rclpy.Parameter.Type.INTEGER_ARRAY, self.expected_params['joints']),
            Parameter('end_effector', rclpy.Parameter.Type.INTEGER, self.expected_params['end_effector']),
        ])

        # Verify subscribers for each module
        for mod in self.expected_params['modules']:
            has_joint = mod in self.expected_params['joints']
            has_end_effector = mod == self.expected_params['end_effector']
            subs = node.create_module_subs(mod, has_joint, has_end_effector)
            for topic in node.topics_from_direction(rc.Direction.OUT):
                if topic.name.split("/")[0] == "joint" and not has_joint:
                    continue
                if topic.name.split("/")[0] == "end_effector" and not has_end_effector:
                    continue
                self.assertIn(topic.name, subs, f"Subscriber not found for {topic.name}")


if __name__ == '__main__':
    unittest.main()
