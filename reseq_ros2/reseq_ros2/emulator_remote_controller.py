import select
import sys
import termios
import traceback
import tty

import rclpy
from rclpy.node import Node

from reseq_interfaces.msg import Remote

# retrieve settings from file descriptor (stdin)
settings = termios.tcgetattr(sys.stdin)

class EmulatorRemoteController(Node):
    def __init__(self):
        super().__init__("emulator_remote_controller")
        self.publisher = self.create_publisher(Remote, '/remote', 10)
        self.timer = self.create_timer(0.08, self.readLoop)
        self.increment = 0.1
        self.previousMessage = Remote()
        self.get_logger().info("EmulatorRemoteController node started")

    def readKey(self):
        tty.setraw(sys.stdin.fileno())
        # add timeout, so select.select() will wait for that time and then it will return
        # it returns three lists one for each of the three parameters
        readyToRead, _,_ = select.select([sys.stdin], [], [], 0.08)
        if readyToRead:
            key = sys.stdin.read(1)
        else:
            # use default key
            key = ' '
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def handleKey(self, key):
        key = key.lower()

        if key == 'w':
            self.previousMessage.left.y = min(1.0, self.previousMessage.left.y + self.increment)
        elif key == 's':
            self.previousMessage.left.y = max(-1.0, self.previousMessage.left.y - self.increment)
        elif key == 'a':
            self.previousMessage.left.x = max(-1.0, self.previousMessage.left.x - self.increment)
        elif key == 'd':
            self.previousMessage.left.x = min(1.0, self.previousMessage.left.x + self.increment)
        elif key == 'q':
            self.previousMessage.left.z = max(-1.0, self.previousMessage.left.z - self.increment)
        elif key == 'e':
            self.previousMessage.left.z = min(1.0, self.previousMessage.left.z + self.increment)
        elif key == 'i':
            self.previousMessage.right.y = min(1.0, self.previousMessage.right.y + self.increment)
        elif key == 'k':
            self.previousMessage.right.y = max(-1.0, self.previousMessage.right.y - self.increment)
        elif key == 'j':
            self.previousMessage.right.x = max(-1.0, self.previousMessage.right.x - self.increment)
        elif key == 'l':
            self.previousMessage.right.x = min(1.0, self.previousMessage.right.x + self.increment)
        elif key == 'b':
            self.increment  *= 2
        elif key == 'h':
            self.increment /= 2
        elif key == 'r':
            self.previousMessage = Remote()
            self.increment = 0.1
        elif key == 'z':
            raise Exception("Exit requested")
        self.publisher.publish(self.previousMessage)

    def readLoop(self):
        key = self.readKey()
        self.handleKey(key)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = EmulatorRemoteController()
    except Exception as err:
        rclpy.logging.get_logger('enea').fatal(
            f'Error in the Enea node: {str(err)}\n{traceback.format_exc()}'
        )
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        raise err
    else:
        rclpy.spin(node)
        node.destroy_node()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == "__main__":
    main()
