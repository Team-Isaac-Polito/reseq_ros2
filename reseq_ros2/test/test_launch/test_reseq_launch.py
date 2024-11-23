import os
import unittest
import pytest
import sys
import rclpy
import re

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_testing.actions import ReadyToTest
import time

from ament_index_python.packages import get_package_share_directory

# Default config file path
share_folder = get_package_share_directory("reseq_ros2")
sys.path.append(share_folder+"/launch")
from common_functions_launch import *

@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        ExecuteProcess(
                cmd=[
                    'ros2', 'launch', 'reseq_ros2', 'reseq_launch.py', 'config_file:=reseq_mk1_vcan.yaml',
                ],
        ),
        ReadyToTest()
    ])

# Test

class TestReseqLaunch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_node')
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_1_nodes_up(self):
        """Test that all the expected nodes are up and running."""
        nodes = ['communication',
                 'agevar',
                 'scaler',
                 'enea',
                 'realsense2_camera_node',
                 'rplidar_node',
                 'joint_publisher',
                 'controller_manager',
                 'joint_state_broadcaster',
                 'diff_controller3',
                 'diff_controller2',
                 'diff_controller1',
                 'robot_state_publisher']
        nodes_down = []

        time.sleep(5)
        end_time = time.time() + 10
        while time.time() < end_time:
            exist_nodes = self.node.get_node_names()
            for node in nodes:
                if node not in exist_nodes: nodes_down.append(node)
            time.sleep(1)
        # Assert and print the nodes that are not running
        self.assertTrue(len(nodes_down) == 0, f"The following nodes are not running: {', '.join(set(nodes_down))}")

    def test_2_no_crash(self):
        """Test that no nodes exited with an error."""
        log_root_directory = os.path.expanduser('~/.ros/log')
        latest_log_directory = max(
            [os.path.join(log_root_directory, d) for d in os.listdir(log_root_directory) if os.path.isdir(os.path.join(log_root_directory, d))],
            key=os.path.getmtime
        )
        log_file_path = os.path.join(latest_log_directory, 'launch.log')

        with open(log_file_path, 'r') as log_file:
            logs = log_file.read()

        # Check for "Node has died" error log using regex for any node
        error_pattern = re.compile(r"\[ERROR\] \[[a-zA-Z0-9_]+-\d+\]: process has died")
        errors_found = []
        for line in logs.splitlines():
            if error_pattern.search(line):
                errors_found.append(line + '\n')
        # Assert if any errors were found, including the exact error messages
        self.assertTrue(len(errors_found) == 0, f"Processes have died during the test:\n{''.join(errors_found)}")

if __name__ == '__main__':
    unittest.main()
