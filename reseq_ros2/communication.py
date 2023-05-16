import rclpy
import can
from can import Message
from rclpy.node import Node


class Communication(Node):
    def __init__(self):
        super().__init__('communication')

        # canbus
        self.canbus = can.interface.Bus(channel='vcan0', bustype='socketcan')
        self.notifier = can.Notifier(self.canbus, [self.can_callback])

        #TODO: define ROS publishers and subscribers

        self.get_logger().info("Communication node started")

    # publish data received from CAN to ROS topic (prev. recv_data)
    def can_callback(self, msg):
        print(msg)
        pass

    # publish data received from ROS to CAN
    def ros_listener_callback(self, msg, module_num, topic):
        # based on the topic the message was published to
        # package data into bytearray
        # write arbitration id (from constants)
        pass


def main(args=None):
    rclpy.init(args=args)
    communication = Communication()
    rclpy.spin(communication)
    communication.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
