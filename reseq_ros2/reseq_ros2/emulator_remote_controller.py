import sys
import select
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
        self.timer = self.create_timer(0.08, self.readLoop)
        self.has_to_exit = False
        self.previousMessage = Remote()
        self.increment = 0.1

    '''
        The following function is the adaptation of the one linked below:
        https://github.com/ros2/teleop_twist_keyboard/blob/8a2b696be2461373d6b573eb7a504a9161307b57/teleop_twist_keyboard.py#L105-L114
    '''
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

    def handleKey(self, key, message=Remote()):
        # make lower case
        key = key.lower()
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
            self.previousMessage.right.x += self.increment
            self.previousMessage.right.x = min(1.0, self.previousMessage.right.x)
            self.publisher.publish(self.previousMessage)
        # turn left
        elif key == 'j':
            self.previousMessage.right.x -= self.increment
            self.previousMessage.right.x = max(-1.0, self.previousMessage.right.x)
            self.publisher.publish(self.previousMessage)
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

    def readLoop(self):
        # create a default message
        message = Remote()
        key = self.readKey()
        if self.handleKey(key, message):
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
        while (rclpy.ok() and not emulator_remote_controller.has_to_exit):
            rclpy.spin_once(emulator_remote_controller)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
