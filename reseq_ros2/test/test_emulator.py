import unittest
import rclpy
from unittest.mock import MagicMock
from reseq_ros2.emulator_remote_controller import EmulatorRemoteController
from reseq_ros2.emulator_monitor import EmulatorMonitor

class TestEmulator(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # set up resources shared for all tests
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # run emulator_remote_controller and a node to read from remote
        self.sender = EmulatorRemoteController()
        self.receiver = EmulatorMonitor()

    def tearDown(self):
        self.sender.emulator.destroy_node()
        self.sender.destroy_node()
        self.receiver.destroy_node()

    # keys are WASDQE and JKLI and HB
    def test_w(self):
        # use MagicMock instead of directly passing char to function, since it wouldn't be able to understand
        # the type
        key = MagicMock()
        key = 'w'
        self.sender.handleKey(key)
        self.assertEqual(self.sender.state, 'Normal')
        self.assertEqual(self.sender.previousKey, 'w')
        self.assertEqual(self.sender.previousValue, 0.1)
        # if you send, always spin_once the receiver
        rclpy.spin_once(self.receiver)
        self.assertEqual(self.receiver.msg.left.y, 0.1)

        self.sender.handleKey(key, )
        self.assertEqual(self.sender.previousKey, 'w')
        self.assertEqual(round(self.sender.previousValue,3), 0.110)
        rclpy.spin_once(self.receiver)
        # loop many times to check the max is 1
        for _ in range(26):
            self.sender.handleKey(key)
            rclpy.spin_once(self.receiver)
        self.assertEqual(round(self.sender.previousValue,3), 1)
        self.assertEqual(round(self.receiver.msg.left.y,3), 1)

    def test_s(self):
        # use MagicMock instead of directly passing char to function, since it wouldn't be able to understand
        # the type
        key = MagicMock()
        key = 's'
        self.sender.handleKey(key)
        self.assertEqual(self.sender.state, 'Normal')
        self.assertEqual(self.sender.previousKey, 's')
        self.assertEqual(self.sender.previousValue, -0.1)
        # if you send, always spin_once the receiver
        rclpy.spin_once(self.receiver)
        self.assertEqual(self.receiver.msg.left.y, -0.1)

        self.sender.handleKey(key)
        self.assertEqual(self.sender.previousKey, 's')
        self.assertEqual(round(self.sender.previousValue,3), -0.110)
        rclpy.spin_once(self.receiver)
        # loop many times to check the max is 1
        for _ in range(26):
            self.sender.handleKey(key)
            rclpy.spin_once(self.receiver)
        self.assertEqual(round(self.sender.previousValue,3), -1)
        self.assertEqual(round(self.receiver.msg.left.y,3), -1)
    
    def test_a(self):
        key = MagicMock()
        key = 'A'
        self.sender.handleKey(key)
        self.assertEqual(self.sender.state, 'Normal')
        self.assertEqual(self.sender.previousKey, 'a')
        self.assertEqual(self.sender.previousValue, -0.1)
        # if you send, always spin_once the receiver
        rclpy.spin_once(self.receiver)
        self.assertEqual(self.receiver.msg.left.x, -0.1)

        self.sender.handleKey(key)
        self.assertEqual(self.sender.previousKey, 'a')
        self.assertEqual(round(self.sender.previousValue,3), -0.110)
        rclpy.spin_once(self.receiver)
        # loop many times to check the max is 1
        for _ in range(26):
            self.sender.handleKey(key)
            rclpy.spin_once(self.receiver)
        self.assertEqual(round(self.sender.previousValue,3), -1)
        self.assertEqual(round(self.receiver.msg.left.x,3), -1)

    def test_d(self):
        # use MagicMock instead of directly passing char to function, since it wouldn't be able to understand
        # the type
        key = MagicMock()
        key = 'd'
        self.sender.handleKey(key)
        self.assertEqual(self.sender.state, 'Normal')
        self.assertEqual(self.sender.previousKey, 'd')
        self.assertEqual(self.sender.previousValue, 0.1)
        # if you send, always spin_once the receiver
        rclpy.spin_once(self.receiver)
        self.assertEqual(self.receiver.msg.left.x, 0.1)

        self.sender.handleKey(key)
        self.assertEqual(self.sender.previousKey, 'd')
        self.assertEqual(round(self.sender.previousValue,3), 0.110)
        rclpy.spin_once(self.receiver)
        # loop many times to check the max is 1
        for _ in range(26):
            self.sender.handleKey(key)
            rclpy.spin_once(self.receiver)
        self.assertEqual(round(self.sender.previousValue,3), 1)
        self.assertEqual(round(self.receiver.msg.left.x,3), 1)

    def test_q(self):
        key = MagicMock()
        key = 'q'
        self.sender.handleKey(key)
        self.assertEqual(self.sender.state, 'Normal')
        self.assertEqual(self.sender.previousKey, 'q')
        self.assertEqual(self.sender.previousValue, -0.1)
        # if you send, always spin_once the receiver
        rclpy.spin_once(self.receiver)
        self.assertEqual(self.receiver.msg.left.z, -0.1)

        self.sender.handleKey(key)
        self.assertEqual(self.sender.previousKey, 'q')
        self.assertEqual(round(self.sender.previousValue,3), -0.110)
        rclpy.spin_once(self.receiver)
        # loop many times to check the max is 1
        for _ in range(26):
            self.sender.handleKey(key)
            rclpy.spin_once(self.receiver)
        self.assertEqual(round(self.sender.previousValue,3), -1)
        self.assertEqual(round(self.receiver.msg.left.z,3), -1)

    def test_e(self):
        key = MagicMock()
        key = 'e'
        self.sender.handleKey(key)
        self.assertEqual(self.sender.state, 'Normal')
        self.assertEqual(self.sender.previousKey, 'e')
        self.assertEqual(self.sender.previousValue, 0.1)
        # if you send, always spin_once the receiver
        rclpy.spin_once(self.receiver)
        self.assertEqual(self.receiver.msg.left.z, 0.1)

        self.sender.handleKey(key)
        self.assertEqual(self.sender.previousKey, 'e')
        self.assertEqual(round(self.sender.previousValue,3), 0.110)
        rclpy.spin_once(self.receiver)
        # loop many times to check the max is 1
        for _ in range(26):
            self.sender.handleKey(key)
            rclpy.spin_once(self.receiver)
        self.assertEqual(round(self.sender.previousValue,3), 1)
        self.assertEqual(round(self.receiver.msg.left.z,3), 1)

    def test_i(self):
        key = MagicMock()
        key = 'i'
        self.sender.handleKey(key)
        self.assertEqual(self.sender.state, 'Normal')
        self.assertEqual(self.sender.previousKey, 'i')
        self.assertEqual(self.sender.previousValue, 0.1)
        # if you send, always spin_once the receiver
        rclpy.spin_once(self.receiver)
        self.assertEqual(self.receiver.msg.right.y, 0.1)

        self.sender.handleKey(key)
        self.assertEqual(self.sender.previousKey, 'i')
        self.assertEqual(round(self.sender.previousValue,3), 0.110)
        rclpy.spin_once(self.receiver)
        # loop many times to check the max is 1
        for _ in range(26):
            self.sender.handleKey(key)
            rclpy.spin_once(self.receiver)
        self.assertEqual(round(self.sender.previousValue,3), 1)
        self.assertEqual(round(self.receiver.msg.right.y,3), 1)

    def test_k(self):
        key = MagicMock()
        key = 'k'
        self.sender.handleKey(key)
        self.assertEqual(self.sender.state, 'Normal')
        self.assertEqual(self.sender.previousKey, 'k')
        self.assertEqual(self.sender.previousValue, -0.1)
        # if you send, always spin_once the receiver
        rclpy.spin_once(self.receiver)
        self.assertEqual(self.receiver.msg.right.y, -0.1)

        self.sender.handleKey(key)
        self.assertEqual(self.sender.previousKey, 'k')
        self.assertEqual(round(self.sender.previousValue,3), -0.110)
        rclpy.spin_once(self.receiver)
        # loop many times to check the max is 1
        for _ in range(26):
            self.sender.handleKey(key)
            rclpy.spin_once(self.receiver)
        self.assertEqual(round(self.sender.previousValue,3), -1)
        self.assertEqual(round(self.receiver.msg.right.y,3), -1)
    
    def test_l(self):
        key = MagicMock()
        key = 'l'
        self.sender.handleKey(key)
        self.assertEqual(self.sender.state, 'Normal')
        self.assertEqual(self.sender.previousKey, 'l')
        self.assertEqual(self.sender.previousValue, 0.1)
        # if you send, always spin_once the receiver
        rclpy.spin_once(self.receiver)
        self.assertEqual(self.receiver.msg.right.x, 0.1)

        self.sender.handleKey(key)
        self.assertEqual(self.sender.previousKey, 'l')
        self.assertEqual(round(self.sender.previousValue,3), 0.110)
        rclpy.spin_once(self.receiver)
        # loop many times to check the max is 1
        for _ in range(26):
            self.sender.handleKey(key)
            rclpy.spin_once(self.receiver)
        self.assertEqual(round(self.sender.previousValue,3), 1)
        self.assertEqual(round(self.receiver.msg.right.x,3), 1)

    def test_j(self):
        key = MagicMock()
        key = 'j'
        self.sender.handleKey(key)
        self.assertEqual(self.sender.state, 'Normal')
        self.assertEqual(self.sender.previousKey, 'j')
        self.assertEqual(self.sender.previousValue, -0.1)
        # if you send, always spin_once the receiver
        rclpy.spin_once(self.receiver)
        self.assertEqual(self.receiver.msg.right.x, -0.1)

        self.sender.handleKey(key)
        self.assertEqual(self.sender.previousKey, 'j')
        self.assertEqual(round(self.sender.previousValue,3), -0.110)
        rclpy.spin_once(self.receiver)
        # loop many times to check the max is 1
        for _ in range(26):
            self.sender.handleKey(key)
            rclpy.spin_once(self.receiver)
        self.assertEqual(round(self.sender.previousValue,3), -1)
        self.assertEqual(round(self.receiver.msg.right.x,3), -1)

    def test_bh_exchange(self):
        key = MagicMock()
        key_m = MagicMock()
        key_m = 'i'
        key = 'h'
        self.sender.handleKey(key_m)
        self.assertEqual(self.sender.state, 'Normal')
        self.assertEqual(self.sender.previousKey, 'i')
        self.assertEqual(self.sender.previousValue, 0.1)
        # if you send, always spin_once the receiver
        rclpy.spin_once(self.receiver)
        self.assertEqual(self.receiver.msg.right.y, 0.1)

        self.sender.handleKey(key)
        self.sender.handleKey(key_m)
        self.assertEqual(self.sender.state, 'Double')
        self.assertEqual(self.sender.previousKey, 'i')
        self.assertEqual(self.sender.previousValue, 0.12)
        # if you send, always spin_once the receiver
        rclpy.spin_once(self.receiver)
        self.assertEqual(self.receiver.msg.right.y, 0.12)

        self.sender.handleKey(key)
        self.sender.handleKey(key_m)
        self.assertEqual(self.sender.previousKey, 'i')
        self.assertEqual(round(self.sender.previousValue,3), 0.132)
        rclpy.spin_once(self.receiver)
        self.assertEqual(self.receiver.msg.right.y, 0.132)

    def test_bh(self):
        key = MagicMock()
        key_m = MagicMock()
        key_m = 'i'
        key = 'h'
        self.assertEqual(self.sender.state, 'Normal')
        self.sender.handleKey(key)
        self.assertEqual(self.sender.state, 'Double')
        self.sender.handleKey(key_m)
        self.assertEqual(self.sender.previousKey, 'i')
        self.assertEqual(self.sender.previousValue, 0.2)
        rclpy.spin_once(self.receiver)
        self.assertEqual(self.receiver.msg.right.y, 0.2)

if __name__ == "__main__":
    unittest.main()