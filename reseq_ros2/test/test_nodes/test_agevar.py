import unittest
from math import cos, pi, sin
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
from std_msgs.msg import Float32

import reseq_ros2.constants as rc
from reseq_interfaces.msg import Motors
from reseq_ros2.agevar import Agevar

# Launch the Agevar using the reseq MK1 vcan configurations
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

class TestAgevarNode(unittest.TestCase):
    """Test class for the Agevar node's functionality."""

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)
        cls.executor = SingleThreadedExecutor()
        cls.context = rclpy.get_default_context()
        cls.node = rclpy.create_node('test_node', context=cls.context)
        cls.executor.add_node(cls.node)

        # Expected parameter values from agevar_consts.yaml
        cls.expected_params = {
            'a': 0.1695,
            'b': 0.18395,
            'd': 0.223,
            'r_eq': 0.0465,
            'modules': [0x11, 0x12, 0x13],  # received from reseq_mk1_vcan.yaml
            'joints': [0x12, 0x13],   # received from reseq_mk1_vcan.yaml
            'end_effector': 0x11  # received from reseq_mk1_vcan.yaml
        }

        # Wait for the Agevar node to be up and running
        timeout_sec = 10
        agevar_node_found = False
        start_time = time()
        while time() - start_time < timeout_sec and not agevar_node_found:
            node_names = cls.node.get_node_names()
            agevar_node_found = 'agevar' in node_names
            if not agevar_node_found:
                rclpy.spin_once(cls.node, timeout_sec=1)

        if not agevar_node_found:
            raise RuntimeError('Agevar node was not found running within the timeout period.')

        # Create a client to get parameters of the agevar node
        cls.parameter_client = cls.node.create_client(GetParameters, '/agevar/get_parameters')

        if not cls.parameter_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError('Agevar node parameter service not available within the timeout period.')
        
        # Create a client to list all parameters of the agevar node
        cls.list_parameters_client = cls.node.create_client(ListParameters, '/agevar/list_parameters')

        if not cls.list_parameters_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError('Agevar node list parameters service not available within the timeout period.')
        
        # Initialize the agevar node
        cls.agevar = Agevar()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        cls.agevar.destroy_node()
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
        all_parameters = self.list_all_parameters('agevar')
        unexpected_parameters = set(all_parameters[1:]) - set(self.expected_params.keys())
        missing_parameters = set(self.expected_params.keys()) - set(all_parameters[1:])

        self.assertEqual(len(unexpected_parameters), 0, f"Unexpected parameters found: {unexpected_parameters}")
        self.assertEqual(len(missing_parameters), 0, f"Missing parameters found: {missing_parameters}")

        for param_name, expected_value in self.expected_params.items():
            param_value = self.get_parameter('agevar', param_name)
            self.assertEqual(param_value, expected_value, f"{param_name} parameter value mismatch")

    def test_2_vel_motors(self):
        """Test the vel_motors function."""
        node = self.agevar

        # Set initial values and constraints
        node.d = self.expected_params['d']
        node.r_eq = self.expected_params['r_eq']

        lin_vel = -0.5
        ang_vel = 0.1
        sign = (lin_vel > 0)

        # Call vel_motors
        w_right, w_left = node.vel_motors(lin_vel, ang_vel, sign)

        # Calculate expected values
        expected_w_right = (lin_vel + ang_vel * node.d / 2) / node.r_eq * rc.rads2rpm
        expected_w_left = (lin_vel - ang_vel * node.d / 2) / node.r_eq * rc.rads2rpm
        if sign == 0:
            expected_w_left, expected_w_right = -expected_w_left, -expected_w_right

        # Verify motor velocities
        self.assertAlmostEqual(w_right, expected_w_right, places=5)
        self.assertAlmostEqual(w_left, expected_w_left, places=5)

    def test_3_kinematic(self):
        """Test the kinematic function."""
        node = self.agevar

        # Set initial values and constraints
        node.a = self.expected_params['a']
        node.b = self.expected_params['b']

        linear_vel = 0.5
        angular_vel = 0.1
        yaw_angle = pi / 4

        # Call kinematic
        linear_out, angular_out = node.kinematic(linear_vel, angular_vel, yaw_angle)

        # Calculate expected values
        expected_linear_out = linear_vel * cos(yaw_angle) + node.a * angular_vel * sin(yaw_angle)
        expected_angular_out = (linear_vel * sin(yaw_angle) - node.a * angular_vel * cos(yaw_angle)) / node.b

        # Verify kinematic values
        self.assertAlmostEqual(linear_out, expected_linear_out, places=5)
        self.assertAlmostEqual(angular_out, expected_angular_out, places=5)

    def test_4_yaw_feedback_callback(self):
        """Test the yaw_feedback_callback function."""
        node = self.agevar

        angle_msg = Float32()
        module_num = 17

        # Test case: values within constraints
        angle_msg.data = 45.0

        # Call yaw_feedback_callback
        node.yaw_feedback_callback(angle_msg, module_num)

        # Verify constrained values
        expected_angle = pi/4
        self.assertAlmostEqual(node.yaw_angles[module_num - 17], expected_angle)

        # Test case: values not within constraints
        angle_msg.data = 180.0

        # Call yaw_feedback_callback
        node.yaw_feedback_callback(angle_msg, module_num)

        # Verify constrained values
        expected_angle = -pi
        self.assertAlmostEqual(node.yaw_angles[module_num - 17], expected_angle)

    def test_5_remote_callback(self):
        """Test the remote_callback function."""
        node = self.agevar

        # Set initial state
        node.set_parameters([
            Parameter('a', rclpy.Parameter.Type.DOUBLE, self.expected_params['a']),
            Parameter('b', rclpy.Parameter.Type.DOUBLE, self.expected_params['b']),
            Parameter('d', rclpy.Parameter.Type.DOUBLE, self.expected_params['d']),
            Parameter('r_eq', rclpy.Parameter.Type.DOUBLE, self.expected_params['r_eq']),
            Parameter('modules', rclpy.Parameter.Type.INTEGER_ARRAY, self.expected_params['modules']),
            Parameter('joints', rclpy.Parameter.Type.INTEGER_ARRAY, self.expected_params['joints']),
            Parameter('end_effector', rclpy.Parameter.Type.INTEGER, self.expected_params['end_effector']),
        ])

        twist_msg = Twist()
        twist_msg.linear.x = 0.5
        twist_msg.angular.z = 0.1

        # Additional edge case tests
        edge_case_twist_msgs = [twist_msg]

        twist_msg = Twist()
        twist_msg.linear.x = -0.5
        twist_msg.angular.z = 0.1
        edge_case_twist_msgs.append(twist_msg)

        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        edge_case_twist_msgs.append(twist_msg)

        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.angular.z = -1.0
        edge_case_twist_msgs.append(twist_msg)

        # Capture published messages
        self.motors_msgs = []

        def motors_callback(msg):
            self.motors_msgs.append(msg)

        for module in node.modules:
            node.create_subscription(Motors, f'reseq/module{module}/motor/setpoint', motors_callback, 10)

        # Test all cases
        for edge_case_msg in edge_case_twist_msgs:
            # Clear the messages list for each test case
            self.motors_msgs.clear()
            
            # Call the remote_callback
            node.remote_callback(edge_case_msg)

            # Spin the node to process the publishing and subscription
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.1)
                if len(self.motors_msgs) >= len(node.modules):
                    break
            
            # Verify motors messages
            self.assertEqual(len(self.motors_msgs), len(node.modules))
            lin_vel = edge_case_msg.linear.x
            ang_vel = edge_case_msg.angular.z
            sign = (lin_vel > 0)
            
            if not sign: # Handle the backward case by reversing the velocities
                lin_vel, ang_vel = -lin_vel, -ang_vel
            
            for msg in self.motors_msgs:
                expected_w_right, expected_w_left = node.vel_motors(lin_vel, ang_vel, sign)
                self.assertAlmostEqual(msg.right, expected_w_right, places=4)
                self.assertAlmostEqual(msg.left, expected_w_left, places=4)


if __name__ == '__main__':
    unittest.main()

