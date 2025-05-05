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

# Launch the Scaler using the reseq MK1 vcan configurations
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


class TestScalerLaunch(unittest.TestCase):
    """
    Test class for verifying that the Scaler node
    launches correctly with the expected parameters.
    """

    @classmethod
    def setUpClass(cls):
        cls.process = simulate_launch_test(sys.argv[0])

        rclpy.init(args=None)
        cls.node = rclpy.create_node('test_node')
        cls.expected_node = 'scaler'

        # Expected parameter values from scaler_consts.yaml
        cls.expected_params = {
            'r_linear_vel': [-0.27017, 0.27017],
            'r_inverse_radius': [-1.5385, 1.5385],
            'r_angular_vel': [-2.8387, 2.8387],
            'version': 'mk1',
            'r_pitch_vel': [-183, 183],
            'r_head_pitch_vel': [-400, 400],
            'r_head_roll_vel': [-455, 455],
        }
        cls.timeout_sec = 10

    @classmethod
    def tearDownClass(cls):
        tear_down_process(cls.process)

        cls.node.destroy_node()
        rclpy.shutdown()

    def test_1_nodes_up(self):
        """Test that the Scaler node is up and running."""
        success, message = check_node_up(self.node, self.expected_node, self.timeout_sec)
        self.assertTrue(success, message)

    def test_2_parameters_set(self):
        """Test that parameters are correctly set on the Scaler node."""
        success, message = check_parameters(
            self.node,
            f'/{self.expected_node}/get_parameters',
            self.expected_params,
            self.timeout_sec,
        )
        self.assertTrue(success, message)

    def test_3_missing_parameters(self):
        """Test for unexpected or missing parameters on the Scaler node."""
        success, message = check_missing_parameters(
            self.node,
            f'/{self.expected_node}/list_parameters',
            self.expected_params,
            self.timeout_sec,
        )
        self.assertTrue(success, message)


if __name__ == '__main__':
    unittest.main()
