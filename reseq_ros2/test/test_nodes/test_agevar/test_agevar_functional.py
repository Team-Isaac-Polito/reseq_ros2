import unittest
from math import cos, pi, sin

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import Float32

import reseq_ros2.constants as rc
from reseq_interfaces.msg import Motors
from reseq_ros2.agevar import Agevar

# Function Test


class TestAgevarFunctional(unittest.TestCase):
    """Test class for the Agevar node's functionality."""

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)
        cls.executor = SingleThreadedExecutor()
        cls.agevar = Agevar()
        cls.executor.add_node(cls.agevar)

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

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.agevar.destroy_node()
        rclpy.shutdown()

    def test_4_init_conditions(self):
        node = self.agevar

        node.a = self.expected_params['a']
        node.b = self.expected_params['b']

        node.init_conditions()
        for m in range(node.n_mod):
            self.assertEqual(node.eta[m], [0.0, (m - 1) * (-node.a - node.b), 0.0])
            self.assertEqual(node.etad[m], [0.0, 0.0, 0.0])

    def test_5_vel_motors(self):
        """Test the vel_motors function."""
        node = self.agevar

        # Set initial values and constraints
        node.d = self.expected_params['d']
        node.r_eq = self.expected_params['r_eq']

        lin_vel = -0.5
        ang_vel = 0.1
        sign = lin_vel > 0

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

    def test_6_kinematic(self):
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
        expected_angular_out = (
            linear_vel * sin(yaw_angle) - node.a * angular_vel * cos(yaw_angle)
        ) / node.b

        # Verify kinematic values
        self.assertAlmostEqual(linear_out, expected_linear_out, places=5)
        self.assertAlmostEqual(angular_out, expected_angular_out, places=5)

    def test_7_yaw_feedback_callback(self):
        """Test the yaw_feedback_callback function."""
        node = self.agevar

        angle_msg = Float32()
        module_num = 17

        # Test case: values within constraints
        angle_msg.data = 45.0

        # Call yaw_feedback_callback
        node.yaw_feedback_callback(angle_msg, module_num)

        # Verify constrained values
        expected_angle = pi / 4
        self.assertAlmostEqual(node.eta[module_num - 17][0], expected_angle)

        # Test case: values not within constraints
        angle_msg.data = 180.0

        # Call yaw_feedback_callback
        node.yaw_feedback_callback(angle_msg, module_num)

        # Verify constrained values
        expected_angle = -pi
        self.assertAlmostEqual(node.eta[module_num - 17][0], expected_angle)

    def test_8_rotz(self):
        """Test the Rotz function."""
        node = self.agevar

        # Test case: 90 degrees (pi/2 radians)
        theta = pi / 2
        expected_matrix = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        result_matrix = node.Rotz(theta)
        np.testing.assert_array_almost_equal(result_matrix, expected_matrix, decimal=5)

        # Test case: -90 degrees (-pi/2 radians)
        theta = -pi / 2
        expected_matrix = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        result_matrix = node.Rotz(theta)
        np.testing.assert_array_almost_equal(result_matrix, expected_matrix, decimal=5)

        # Test case: 180 degrees (pi radians)
        theta = pi
        expected_matrix = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
        result_matrix = node.Rotz(theta)
        np.testing.assert_array_almost_equal(result_matrix, expected_matrix, decimal=5)

        # Test case: -180 degrees (-pi radians)
        theta = -pi
        expected_matrix = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
        result_matrix = node.Rotz(theta)
        np.testing.assert_array_almost_equal(result_matrix, expected_matrix, decimal=5)

        # Test case: 0 degrees (0 radians)
        theta = 0
        expected_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        result_matrix = node.Rotz(theta)
        np.testing.assert_array_almost_equal(result_matrix, expected_matrix, decimal=5)

    def test_9_remote_callback(self):
        """Test the remote_callback function."""
        node = self.agevar

        # Set initial state
        node.set_parameters(
            [
                Parameter('a', rclpy.Parameter.Type.DOUBLE, self.expected_params['a']),
                Parameter('b', rclpy.Parameter.Type.DOUBLE, self.expected_params['b']),
                Parameter('d', rclpy.Parameter.Type.DOUBLE, self.expected_params['d']),
                Parameter('r_eq', rclpy.Parameter.Type.DOUBLE, self.expected_params['r_eq']),
                Parameter(
                    'modules', rclpy.Parameter.Type.INTEGER_ARRAY, self.expected_params['modules']
                ),
                Parameter(
                    'joints', rclpy.Parameter.Type.INTEGER_ARRAY, self.expected_params['joints']
                ),
                Parameter(
                    'end_effector',
                    rclpy.Parameter.Type.INTEGER,
                    self.expected_params['end_effector'],
                ),
            ]
        )

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
            node.create_subscription(
                Motors, f'reseq/module{module}/motor/setpoint', motors_callback, 10
            )

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
            sign = lin_vel > 0

            if not sign:  # Handle the backward case by reversing the velocities
                lin_vel, ang_vel = -lin_vel, -ang_vel

            for msg in self.motors_msgs:
                expected_w_right, expected_w_left = node.vel_motors(lin_vel, ang_vel, sign)
                self.assertAlmostEqual(msg.right, expected_w_right, places=4)
                self.assertAlmostEqual(msg.left, expected_w_left, places=4)


if __name__ == '__main__':
    unittest.main()
