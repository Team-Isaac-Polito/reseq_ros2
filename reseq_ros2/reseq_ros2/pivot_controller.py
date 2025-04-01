import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from reseq_interfaces.msg import Motors, Remote


class PivotController(Node):
    def __init__(self):
        super().__init__('pivot_controller')
        self.subscription = self.create_subscription(Remote, '/remote', self.remote_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # publishers for direct motor control (otherwise all modules start moving)
        self.motor_pub_module_first = self.create_publisher(
            Motors, '/reseq/module17/motor/setpoint', 10
        )
        self.motor_pub_module_last = self.create_publisher(
            Motors, '/reseq/module19/motor/setpoint', 10
        )

        self.increment = 0.1
        self.pivot_on_first_module = True
        self.is_pivot_mode = False
        self.prev_blue_pressed = False

    def remote_callback(self, msg: Remote):
        buttons = msg.buttons if len(msg.buttons) >= 6 else [False] * 6
        blue_pressed = buttons[5]
        joystick_active = abs(msg.right.x) > 0.001

        self.get_logger().info(f'[REMOTE] right.x = {msg.right.x}')

        if blue_pressed and not self.prev_blue_pressed:
            self.get_logger().info('Pivot mode activated')
            self.stop_agevar()

            if len(buttons) > 4:
                self.pivot_on_first_module = not buttons[4]
                self.get_logger().info(
                    f'Pivoting on {"first" if self.pivot_on_first_module else "last"} module '
                    f'(switch state = {buttons[4]})'
                )
            self.is_pivot_mode = True

        elif not blue_pressed and self.prev_blue_pressed:
            self.get_logger().info('Pivot mode deactivated')
            self.resume_agevar()
            self.is_pivot_mode = False

        if self.is_pivot_mode and blue_pressed and joystick_active:
            self.handle_pivot(msg.right.x)

        if self.is_pivot_mode and len(buttons) > 4:
            new_state = not buttons[4]
            if new_state != self.pivot_on_first_module:
                self.pivot_on_first_module = new_state
                self.get_logger().info(
                    'Pivot target changed to '
                    f'{"first" if self.pivot_on_first_module else "last"} module '
                    f'(switch state = {buttons[4]})'
                )

        self.prev_blue_pressed = blue_pressed

    def resume_agevar(self):
        resume_msg = Twist()
        resume_msg.linear.x = 0.001
        self.cmd_vel_pub.publish(resume_msg)
        self.get_logger().info('Published resume command to /cmd_vel to resume AGeVaR.')

    def stop_agevar(self):
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info('Published zero velocity to /cmd_vel to stop AGeVaR.')

    def handle_pivot(self, joystick_value):
        self.get_logger().info(f'[PIVOT] Joystick value: {joystick_value}')
        delta = max(min(joystick_value, 1.0), -1.0) * self.increment

        pivot_msg = Motors()
        pivot_msg.left = -delta
        pivot_msg.right = delta

        if self.pivot_on_first_module:
            self.motor_pub_module_first.publish(pivot_msg)
        else:
            self.motor_pub_module_last.publish(pivot_msg)


def main(args=None):
    rclpy.init(args=args)
    pivot_controller = PivotController()
    try:
        rclpy.spin(pivot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        pivot_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
