import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

TS = 1/50  # sampling time
T_IN = 2
T_STEP = 4
T_SIM = 8

"""Script for testing

Publish packets to the same ROS topic used by the remote controller to simulate it
and make sure all components behave correctly.
"""

class RemoteTest(Node):
    def __init__(self):
        super().__init__('remote_test')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(TS, self.timer_callback)
        self.t = 0

    def timer_callback(self):
        msg = Twist()

        if self.t <= T_IN:
            self.get_logger().debug('not moving')
            linear_vel = 0
            turn_radius = 0
        elif self.t <= T_STEP:
            self.get_logger().debug('going straight')
            linear_vel = 500
            turn_radius = 0
        elif self.t <= T_SIM:
            self.get_logger().debug('turning')
            linear_vel = 500
            turn_radius = 500
        else:
            self.get_logger().debug('stopped')
            linear_vel = 0
            turn_radius = 0
            self.destroy_timer(self.timer)
            self.destroy_node()
            exit()

        msg.linear.x = float(linear_vel)
        msg.angular.z = float(turn_radius)

        self.publisher.publish(msg)
        self.get_logger().debug(
            f"Linear velocity: {msg.linear.x}\t Turn radius: {msg.angular.z}")
        self.t += TS

def main(args=None):
    rclpy.init(args=args)
    remote_test = RemoteTest()
    rclpy.spin(remote_test)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
