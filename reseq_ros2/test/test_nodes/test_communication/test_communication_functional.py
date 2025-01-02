import unittest
from test.utils.test_utils import check_interface_status

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

import reseq_ros2.constants as rc
from reseq_ros2.communication import Communication

# Function Test


# Define the interface status and message
interface_status, status_msg = check_interface_status('vcan0')


@unittest.skipIf(not interface_status, reason=status_msg)
class TestCommunicationFunctional(unittest.TestCase):
    """Test class for the Communication node's functionality."""

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)
        cls.executor = SingleThreadedExecutor()
        cls.communication = Communication()
        cls.executor.add_node(cls.communication)

        # Expected parameter values
        cls.expected_params = {
            'can_channel': 'vcan0',
            'modules': [0x11, 0x12, 0x13],  # received from reseq_mk1_vcan.yaml
            'joints': [0x12, 0x13],  # received from reseq_mk1_vcan.yaml
            'end_effector': 0x11,  # received from reseq_mk1_vcan.yaml
        }

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.communication.canbus.shutdown()
        cls.communication.destroy_node()
        rclpy.shutdown()

    def test_4_create_module_pubs(self):
        """Test the create_module_pubs function."""
        node = self.communication

        # Set initial parameters
        node.set_parameters(
            [
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

        # Verify publishers for each module
        for mod in self.expected_params['modules']:
            has_joint = mod in self.expected_params['joints']
            has_end_effector = mod == self.expected_params['end_effector']
            pubs = node.create_module_pubs(mod, has_joint, has_end_effector)
            for topic in node.topics_from_direction(rc.Direction.IN):
                if topic.name.split('/')[0] == 'joint' and not has_joint:
                    continue
                if topic.name.split('/')[0] == 'end_effector' and not has_end_effector:
                    continue
                self.assertIn(topic.name, pubs, f'Publisher not found for {topic.name}')

    def test_5_create_module_subs(self):
        """Test the create_module_subs function."""
        node = self.communication

        # Set initial parameters
        node.set_parameters(
            [
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

        # Verify subscribers for each module
        for mod in self.expected_params['modules']:
            has_joint = mod in self.expected_params['joints']
            has_end_effector = mod == self.expected_params['end_effector']
            subs = node.create_module_subs(mod, has_joint, has_end_effector)
            for topic in node.topics_from_direction(rc.Direction.OUT):
                if topic.name.split('/')[0] == 'joint' and not has_joint:
                    continue
                if topic.name.split('/')[0] == 'end_effector' and not has_end_effector:
                    continue
                self.assertIn(topic.name, subs, f'Subscriber not found for {topic.name}')


if __name__ == '__main__':
    unittest.main()
