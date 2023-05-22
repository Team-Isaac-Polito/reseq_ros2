import rclpy
import reseq_ros2.constants as rc

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

T_IN = 2
T_STEP = 4
T_SIM = 8


class RemoteTest(Node):
    def __init__(self):
        super().__init__('remote_test')
        self.publisher = self.create_publisher(Twist, 'remote/joystick', 10)
        timer_period = rc.Ts
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.t = 0

    def timer_callback(self):
        msg = Twist()

        if self.t <= T_IN:
            print('not moving')
            linear_vel = 0
            turn_radius = 0
        elif self.t <= T_STEP:
            print('going straight')
            linear_vel = 500
            turn_radius = 0
        elif self.t <= T_SIM:
            print('turning')
            linear_vel = 500
            turn_radius = 500
        else:
            print('stopped')
            linear_vel = 0
            turn_radius = 0
            self.destroy_timer(self.timer)
            self.destroy_node()
            exit()

        msg.linear.y = float(linear_vel)
        msg.linear.x = float(turn_radius)

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Linear velocity: {msg.linear.y}\t Turn radius: {msg.linear.x}")
        self.t += rc.Ts


def main(args=None):
    rclpy.init(args=args)
    remote_test = RemoteTest()
    rclpy.spin(remote_test)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
