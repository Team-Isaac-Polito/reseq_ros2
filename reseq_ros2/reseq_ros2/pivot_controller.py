import rclpy
from rclpy.node import Node

from reseq_interfaces.msg import Motors, Remote


class PivotController(Node):
    def __init__(self):
        super().__init__('pivot_controller')
        self.subscription = self.create_subscription(Remote, '/remote', self.remote_callback, 10)

        # Publishers for sending motor commands to individual modules
        # These are targeted directly to avoid affecting other modules
        self.motor_pub_module_first = self.create_publisher(
            Motors, '/reseq/module17/motor/setpoint', 10
        )
        self.motor_pub_module_last = self.create_publisher(
            Motors, '/reseq/module19/motor/setpoint', 10
        )

        # Declare max motor speed (used for scaling pivot velocity range)
        # I think it's 65 but I'm not sure, it's what I read in the outline docs
        self.max_motor_speed = (
            self.declare_parameter('max_motor_speed', 65.0).get_parameter_value().double_value
        )

        # Define range of motor velocities during pivot, based on max speed
        self.pivot_velocity_range = [-self.max_motor_speed, self.max_motor_speed]

        # Controls whether pivoting is on the first or last module
        self.pivot_on_first_module = True

        # Tracks whether pivot mode is currently active
        self.is_pivot_mode = False

        # Used to detect transitions in the blue button state (pressed/released)
        self.prev_blue_pressed = False

        # Timer to periodically resend pivot commands if the joystick is held
        self.timer = self.create_timer(0.1, self.periodic_pivot_update)

        # Stores last joystick velocity to maintain command while pivoting
        self.last_velocity = 0.0

    def remote_callback(self, msg: Remote):
        buttons = msg.buttons if len(msg.buttons) >= 6 else [False] * 6
        blue_pressed = buttons[5]
        joystick_active = abs(msg.right.x) > 0.001  # dead zone threshold

        self.get_logger().info(f'[REMOTE] right.x = {msg.right.x}')

        if blue_pressed and not self.prev_blue_pressed:
            self.get_logger().info('Pivot mode activated')

            # Check toggle button for module selection
            if len(buttons) > 4:
                self.pivot_on_first_module = not buttons[4]
                self.get_logger().info(
                    f'Pivoting on {"first" if self.pivot_on_first_module else "last"} module '
                    f'(switch state = {buttons[4]})'
                )
            self.is_pivot_mode = True

        # Detect falling edge of blue button to deactivate pivot mode
        elif not blue_pressed and self.prev_blue_pressed:
            self.get_logger().info('Pivot mode deactivated')
            self.is_pivot_mode = False

        # While pivot mode is active and blue button is held
        if self.is_pivot_mode and blue_pressed:
            if joystick_active:
                # Handle pivot movement based on joystick input
                self.handle_pivot(msg.right.x)
            else:
                # Stop motors if joystick is idle
                stop_msg = Motors()
                stop_msg.left = 0.0
                stop_msg.right = 0.0
                if self.pivot_on_first_module:
                    self.motor_pub_module_first.publish(stop_msg)
                else:
                    self.motor_pub_module_last.publish(stop_msg)

        # Check for module switch change during pivot mode
        if self.is_pivot_mode and len(buttons) > 4:
            new_state = not buttons[4]
            if new_state != self.pivot_on_first_module:
                # send stop command to the previously active module
                stop_msg = Motors()
                stop_msg.left = 0.0
                stop_msg.right = 0.0

                if self.pivot_on_first_module:
                    self.motor_pub_module_first.publish(stop_msg)
                else:
                    self.motor_pub_module_last.publish(stop_msg)

                # now update the target
                self.pivot_on_first_module = new_state
                self.get_logger().info(
                    'Pivot target changed to '
                    f'{"first" if self.pivot_on_first_module else "last"} module '
                    f'(switch state = {buttons[4]})'
                )

        # Store current blue button state for edge detection in next call
        self.prev_blue_pressed = blue_pressed

    def handle_pivot(self, joystick_value):
        self.get_logger().info(
            f'[PIVOT] Joystick value: {joystick_value}'
        )  # here for debugging purposes

        # Convert joystick value to motor speed using scaling function
        velocity = self.scale(joystick_value, self.pivot_velocity_range)
        self.last_velocity = velocity
        self.get_logger().info(
            f'[PIVOT] Scaled velocity: {velocity}'
        )  # here for debugging purposes

        # Create pivot motor message (opposite directions for pivoting)
        pivot_msg = Motors()
        pivot_msg.left = velocity
        pivot_msg.right = -velocity

        # Publish to the appropriate module
        if self.pivot_on_first_module:
            self.motor_pub_module_first.publish(pivot_msg)
        else:
            self.motor_pub_module_last.publish(pivot_msg)

    # This function is probably not necessary at all, but let's be safe
    def periodic_pivot_update(self):
        if self.is_pivot_mode and abs(self.last_velocity) > 0.01:
            pivot_msg = Motors()
            pivot_msg.left = self.last_velocity
            pivot_msg.right = -self.last_velocity
            if self.pivot_on_first_module:
                self.motor_pub_module_first.publish(pivot_msg)
            else:
                self.motor_pub_module_last.publish(pivot_msg)

    def scale(self, val, scaling_range):
        # Clamp joystick input and scale it to motor speed range
        val = max(min(val, 1.0), -1.0)
        max_speed = scaling_range[1]
        return val * max_speed


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
