import sys
import unittest
from test.utils.test_utils import (
    check_missing_parameters,
    check_node_up,
    check_parameters,
    simulate_launch_test,
    tear_down_process,
)

import rclpy
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_testing.actions import ReadyToTest

# Launch the Agevar using the reseq MK1 vcan configurations
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


class TestAgevarLaunch(unittest.TestCase):
    """
    Test class for verifying that the Agevar node
    launches correctly with the expected parameters.
    """

    @classmethod
    def setUpClass(cls):
        cls.process = simulate_launch_test(sys.argv[0])

        rclpy.init()
        cls.node = rclpy.create_node('test_node')
        cls.expected_node = 'agevar'

        # Expected parameter values from agevar_consts.yaml
        cls.expected_params = {
            'a': 0.1695,
            'b': 0.18395,
            'd': 0.223,
            'r_eq': 0.0465,
            'modules': [0x11, 0x12, 0x13],  # received from reseq_mk1_vcan.yaml
            'joints': [0x12, 0x13],  # received from reseq_mk1_vcan.yaml
            'end_effector': 0x11,  # received from reseq_mk1_vcan.yaml
        }
        cls.timeout_sec = 10

    @classmethod
    def tearDownClass(cls):
        tear_down_process(cls.process)

        cls.node.destroy_node()
        rclpy.shutdown()

    def test_1_nodes_up(self):
        """Test that the Agevar node is up and running."""
        success, message = check_node_up(self.node, self.expected_node, self.timeout_sec)
        self.assertTrue(success, message)

    def test_2_parameters_set(self):
        """Test that parameters are correctly set on the Agevar node."""
        success, message = check_parameters(
            self.node,
            f'/{self.expected_node}/get_parameters',
            self.expected_params,
            self.timeout_sec,
        )
        self.assertTrue(success, message)

    def test_3_missing_parameters(self):
        """Test for unexpected or missing parameters on the Agevar node."""
        success, message = check_missing_parameters(
            self.node,
            f'/{self.expected_node}/list_parameters',
            self.expected_params,
            self.timeout_sec,
        )
        self.assertTrue(success, message)


if __name__ == '__main__':
    unittest.main()
