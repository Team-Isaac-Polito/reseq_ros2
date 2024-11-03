import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import traceback

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
        try:
            self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
            self.timer = self.create_timer(TS, self.timer_callback)
            self.t = 0

            self.get_logger().info("RemoteTest node started successfully")
        except Exception as e:
            self.get_logger().fatal(f'Error during initialization: {str(e)}\n{traceback.format_exc()}')
            raise

    def timer_callback(self):
        try:
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
        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {str(e)}\n{traceback.format_exc()}')

def main(args=None):
    rclpy.init(args=args)
    try:
        remote_test = RemoteTest()
    except Exception as err:
        rclpy.logging.get_logger('remote_test').fatal(f"Error while starting RemoteTest node: {str(err)}\n{traceback.format_exc()}")
        rclpy.shutdown()
    else:
        rclpy.spin(remote_test)
        remote_test.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
