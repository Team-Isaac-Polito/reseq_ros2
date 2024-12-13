import sys
import time
import unittest
from test.utils.test_utils import check_interface_status, simulate_launch_test, tear_down_process

import rclpy
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_testing.actions import ReadyToTest

# Test the launch using the reseq MK1 vcan configurations
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


# Check if the CAN interface is up
interface_status, status_msg = check_interface_status('vcan0')


class TestReseqLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.process = simulate_launch_test(sys.argv[0])

        rclpy.init()
        cls.node = rclpy.create_node('test_node')

    @classmethod
    def tearDownClass(cls):
        tear_down_process(cls.process)

        cls.node.destroy_node()
        rclpy.shutdown()

    def test_1_nodes_up(self):
        """Test that all the expected nodes are up and running."""
        nodes = [
            'agevar',
            'scaler',
            'enea',
            'joint_publisher',
            'controller_manager',
            'joint_state_broadcaster',
            'diff_controller3',
            'diff_controller2',
            'diff_controller1',
            'robot_state_publisher',
        ]
        nodes_down = []

        time.sleep(5)
        end_time = time.time() + 10
        while time.time() < end_time:
            exist_nodes = self.node.get_node_names()
            for node in nodes:
                if node not in exist_nodes and node not in nodes_down:
                    nodes_down.append(node)
            time.sleep(1)

        # Assert and print the nodes that are not running
        self.assertTrue(
            len(nodes_down) == 0,
            f"The following nodes are not running: {', '.join(set(nodes_down))}",
        )


if __name__ == '__main__':
    unittest.main()
