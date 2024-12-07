import os
import subprocess
import sys
import unittest
from time import time

import psutil
import rclpy
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_testing.actions import ReadyToTest
from rcl_interfaces.srv import GetParameters, ListParameters

# Launch the Enea using the reseq MK1 vcan configurations
config_file = 'reseq_mk1_vcan.yaml'


def generate_test_description():
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'launch',
                    'reseq_ros2',
                    'reseq_launch.py',
                    f'config_file:={config_file}',
                ],
            ),
            ReadyToTest(),
        ]
    )


# Launch Test


class TestEneaLaunch(unittest.TestCase):
    """
    Test class for verifying that the Enea node
    launches correctly with the expected parameters.
    """

    @classmethod
    def setUpClass(cls):
        if 'launch_test' in os.path.basename(sys.argv[0]):
            pass
        else:
            cmd = [
                'ros2',
                'launch',
                'reseq_ros2',
                'reseq_launch.py',
                f'config_file:={config_file}',
            ]
            cls.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        rclpy.init()
        cls.node = rclpy.create_node('test_node')

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
        cls.timeout_sec = 10

    @classmethod
    def tearDownClass(cls):
        if 'launch_test' in os.path.basename(sys.argv[0]):
            pass
        else:
            try:
                # Ensure all child processes are terminated
                parent_pid = cls.process.pid
                parent = psutil.Process(parent_pid)
                for child in parent.children():
                    child.terminate()

                _, still_alive = psutil.wait_procs(parent.children(), timeout=5)
                for p in still_alive:
                    p.kill()  # Force kill if it still alive
            except psutil.NoSuchProcess:
                pass  # The process has already exited

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
        """Test that the Enea node is up and running."""
        expected_node = 'enea'

        # Wait for the Enea node to be up and running
        enea_node_found = False
        start_time = time()
        while time() - start_time < self.timeout_sec and not enea_node_found:
            node_names = self.node.get_node_names()
            enea_node_found = expected_node in node_names
            if not enea_node_found:
                rclpy.spin_once(self.node, timeout_sec=1)

        # Validate the state of the node
        self.assertTrue(
            enea_node_found, 'Enea node was not found running within the timeout period.'
        )

    def test_2_parameters_set(self):
        """Test that parameters are correctly set on the Enea node."""
        # Create a client to get parameters of the enea node
        parameter_client = self.node.create_client(GetParameters, '/enea/get_parameters')
        client_found = parameter_client.wait_for_service(timeout_sec=self.timeout_sec)
        self.assertTrue(
            client_found, 'Enea node parameter service not available within the timeout period.'
        )

        for param_name, expected_value in self.expected_params.items():
            request = GetParameters.Request()
            request.names = [param_name]
            future = parameter_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            response = future.result()

            # Verify the response
            self.assertIsNotNone(response, f'Failed to get parameter {param_name} from Enea node')

            param_value = response.values[0]
            self.assertEqual(
                self.extract_value(param_value),
                expected_value,
                f'{param_name} parameter value mismatch',
            )

    def test_3_missing_parameters(self):
        """Test for unexpected or missing parameters on the Enea node."""
        # Create a client to list all parameters of the enea node
        list_parameters_client = self.node.create_client(ListParameters, '/enea/list_parameters')
        client_found = list_parameters_client.wait_for_service(timeout_sec=self.timeout_sec)
        self.assertTrue(
            client_found,
            'Enea node list parameters service not available within the timeout period.',
        )

        request = ListParameters.Request()
        future = list_parameters_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        # Verify the response
        self.assertIsNotNone(response, 'Failed to list parameters from Enea node')

        # all params except the 'use_sim_time' parameter
        all_parameters = response.result.names[1:]
        unexpected_parameters = set(all_parameters) - set(self.expected_params.keys())
        missing_parameters = set(self.expected_params.keys()) - set(all_parameters)

        self.assertEqual(
            len(unexpected_parameters), 0, f'Unexpected parameters found: {unexpected_parameters}'
        )
        self.assertEqual(
            len(missing_parameters), 0, f'Missing parameters found: {missing_parameters}'
        )


if __name__ == '__main__':
    unittest.main()
