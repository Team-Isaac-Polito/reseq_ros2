import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node

from reseq_interfaces.msg import Remote

settings = termios.tcgetattr(sys.stdin)


class EmulatorRemoteController(Node):
    def __init__(self):
        super().__init__('emulator_remote_controller')
        self.publisher = self.create_publisher(Remote, '/remote', 10)
        # create timer to start the function
        self.input_timer = self.create_timer(0.08, self.readLoop)
        self.publish_timer = self.create_timer(0.2, self.publish_message)
        self.has_to_exit = False
        self.previousMessage = Remote()
        self.increment = 0.1

    """
        The following function is the adaptation of the one linked below:
        https://github.com/ros2/teleop_twist_keyboard/blob/8a2b696be2461373d6b573eb7a504a9161307b57/teleop_twist_keyboard.py#L105-L114
    """

    def readKey(self):
        tty.setraw(sys.stdin.fileno())
        # add timeout, so select.select() will wait for that time and then it will return
        # it returns three lists one for each of the three parameters
        readyToRead, _, _ = select.select([sys.stdin], [], [], 0.08)
        if readyToRead:
            key = sys.stdin.read(1)
        else:
            # use default key
            key = ' '
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def handleKey(self, key):
        # make lower case
        key = key.lower()

        # simulates blue button being pressed (toggle logic)
        if key == 'm':
            if len(self.previousMessage.buttons) < 6:
                self.previousMessage.buttons = [False] * 6
            self.previousMessage.buttons[5] = True
            self.publisher.publish(self.previousMessage)
            return self.has_to_exit

        # simulates blue button being released (toggle logic)
        elif key == 'n':
            if len(self.previousMessage.buttons) < 6:
                self.previousMessage.buttons = [False] * 6
            self.previousMessage.buttons[5] = False
            self.publisher.publish(self.previousMessage)
            return self.has_to_exit

        # toggle switch 5 (index 4) – rightmost switch
        elif key == 'u':
            if len(self.previousMessage.buttons) < 6:
                self.previousMessage.buttons = [False] * 6
            current_state = (
                self.previousMessage.buttons[4] if len(self.previousMessage.buttons) > 4 else False
            )
            self.previousMessage.buttons[4] = not current_state
            print(f'Switch 5 (Rightmost) toggled to {self.previousMessage.buttons[4]}')
            self.publisher.publish(self.previousMessage)

        # go forward
        if key == 'w':
            self.previousMessage.left.y += self.increment
            # the value must be smaller or equal to 1
            self.previousMessage.left.y = min(1.0, self.previousMessage.left.y)
            self.publisher.publish(self.previousMessage)
        # go backward
        elif key == 's':
            self.previousMessage.left.y -= self.increment
            self.previousMessage.left.y = max(-1.0, self.previousMessage.left.y)
            self.publisher.publish(self.previousMessage)
        # turn left
        elif key == 'a':
            self.previousMessage.left.x -= self.increment
            self.previousMessage.left.x = max(-1.0, self.previousMessage.left.x)
            self.publisher.publish(self.previousMessage)
        # turn right
        elif key == 'd':
            self.previousMessage.left.x += self.increment
            self.previousMessage.left.x = min(1.0, self.previousMessage.left.x)
            self.publisher.publish(self.previousMessage)
        # counter clockwise
        elif key == 'q':
            self.previousMessage.left.z -= self.increment
            self.previousMessage.left.z = max(-1.0, self.previousMessage.left.z)
            self.publisher.publish(self.previousMessage)
        # clockwise
        elif key == 'e':
            self.previousMessage.left.z += self.increment
            self.previousMessage.left.z = min(1.0, self.previousMessage.left.z)
            self.publisher.publish(self.previousMessage)
        # Start cmd_vel commands
        # go forward
        elif key == 'i':
            self.previousMessage.right.y += self.increment
            self.previousMessage.right.y = min(1.0, self.previousMessage.right.y)
            self.publisher.publish(self.previousMessage)
        # go backward
        elif key == 'k':
            self.previousMessage.right.y -= self.increment
            self.previousMessage.right.y = max(-1.0, self.previousMessage.right.y)
            self.publisher.publish(self.previousMessage)
        # turn right
        elif key == 'l':
            self.previousMessage.right.x = 1.0
            self.publisher.publish(self.previousMessage)
            self.previousMessage.right.x = 0.0  # Reset after sending
        # turn left
        elif key == 'j':
            self.previousMessage.right.x = -1.0
            self.publisher.publish(self.previousMessage)
            self.previousMessage.right.x = 0.0  # Reset after sending
        # if b pressed, make cmd_vel movements faster
        elif key == 'b':
            self.increment /= 2
            self.publisher.publish(self.previousMessage)
        elif key == 'h':
            self.increment *= 2
            self.publisher.publish(self.previousMessage)
        elif key == 'z':
            self.has_to_exit = True
            self.publisher.publish(self.previousMessage)
        else:
            self.publisher.publish(self.previousMessage)
        return self.has_to_exit

    def publish_message(self):
        # only republish if blue button (index 5) is pressed, or any joystick value is non zero.
        # otherwise we're just yelling into the void every 0.2s for no reason, which is rude :D
        # not only that, but it also interferes with the logic that
        # depends on message changes (like detecting when pivoting starts/stops)
        buttons = self.previousMessage.buttons
        while len(buttons) < 6:
            buttons.append(False)

        blue_held = buttons[5]
        active_axes = any(
            [
                abs(self.previousMessage.left.x) > 0.01,
                abs(self.previousMessage.left.y) > 0.01,
                abs(self.previousMessage.left.z) > 0.01,
                abs(self.previousMessage.right.x) > 0.01,
                abs(self.previousMessage.right.y) > 0.01,
                abs(self.previousMessage.right.z) > 0.01,
            ]
        )

        if blue_held or active_axes:
            self.publisher.publish(self.previousMessage)

    def readLoop(self):
        key = self.readKey()
        if self.handleKey(key):
            self.timer.cancel()
            self.destroy_node()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            return


def main(args=None):
    rclpy.init(args=args)
    try:
        emulator_remote_controller = EmulatorRemoteController()
    except Exception as err:
        print('Error while starting EmulatorRemoteController node: ' + str(err))
        raise err
    else:
        # if there were no exceptions in running the section inside try, run the following
        # loop until the exit key is pressed
        while rclpy.ok() and not emulator_remote_controller.has_to_exit:
            rclpy.spin_once(emulator_remote_controller)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
