import select
import sys
import termios
import tty
from contextlib import contextmanager

import rclpy
from rclpy.node import Node

from reseq_interfaces.msg import Remote

BUTTON_NAMES = [
    "S1", "S2", "S3", "S4", "S5",
    "BGREEN", "BBLACK", "BRED", "BWHITE", "BBLUE"
]
BUTTON_COUNT = len(BUTTON_NAMES)
BUTTON_DEFAULTS = [False, False, False, False, False, True, True, True, True, True]

SAMPLE_TIME = 0.08
DEFAULT_INCREMENT = 0.1
AXIS_EPS = 0.01

settings = termios.tcgetattr(sys.stdin.fileno())

@contextmanager
def raw_mode(file):
    old_attrs = termios.tcgetattr(file.fileno())
    try:
        tty.setraw(file.fileno())
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)


class RCEmulator(Node):
    def __init__(self):
        super().__init__('RCEmulator')
        self.publisher = self.create_publisher(Remote, '/remote', 10)
        # create timer to start the function
        self.input_timer = self.create_timer(SAMPLE_TIME, self.readLoop)
        self.publish_timer = self.create_timer(SAMPLE_TIME, self.publish_message)
        self.message = Remote()
        self.message.buttons = BUTTON_DEFAULTS.copy()
        self.increment = DEFAULT_INCREMENT
        self.handlers = {
            'w': lambda: self._incr_axis('left', 'y', 1),
            's': lambda: self._incr_axis('left', 'y', -1),
            'a': lambda: self._incr_axis('left', 'x', -1),
            'd': lambda: self._incr_axis('left', 'x', 1),
            'q': lambda: self._incr_axis('left', 'z', -1),
            'e': lambda: self._incr_axis('left', 'z', 1),
            'i': lambda: self._incr_axis('right', 'y', 1),
            'k': lambda: self._incr_axis('right', 'y', -1),
            'l': lambda: self._incr_axis('right', 'x', 1),
            'j': lambda: self._incr_axis('right', 'x', -1),
            'm': lambda: self._toggle_button('BBLUE'),
            'u': lambda: self._toggle_button('S5'),
            'h': lambda: setattr(self, 'increment', self.increment * 2),
            'b': lambda: setattr(self, 'increment', max(self.increment / 2, 0.01)),
            'z': self._request_exit,
            '\x03': self._request_exit,  # Ctrl-C
        }
        self.exiting = False
        self.print_legend()

    def print_legend(self):
        print('Reading from keyboard and Publishing to /remote')
        print('---------------------------')
        print('Use WASD to control left joystick')
        print('Use IJKL to control right joystick')
        print('Use QE to control left joystick Z axis')
        print('u: Toggle S5 button (Pivot on Head)')
        print('m: Toggle BBLUE button (Enable/Disable Agevar/Pivot)')
        print('h: Double increment (current {})'.format(self.increment))
        print('b: Halve increment (current {})'.format(self.increment))
        print('z or Ctrl-C to quit')
        print('---------------------------')

    """
        The following function is the adaptation of the one linked below:
        https://github.com/ros2/teleop_twist_keyboard/blob/8a2b696be2461373d6b573eb7a504a9161307b57/teleop_twist_keyboard.py#L105-L114
    """

    def readKey(self):
        with raw_mode(sys.stdin):
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:
                key = sys.stdin.read(1)
            else:
                key = ''
        return key
    
    def _request_exit(self):
        msg = Remote()
        msg.buttons = BUTTON_DEFAULTS.copy()
        self.publisher.publish(msg)
        self.exiting = True

    def _clamp(self, value: float) -> float:
        return max(-1.0, min(1.0, value))

    def _incr_axis(self, side: str, axis: str, direction: int):
        vec = getattr(self.message, side)
        old = getattr(vec, axis)
        new = self._clamp(old + direction * self.increment)
        setattr(vec, axis, new)

    def _toggle_button(self, button_name: str):
        index = BUTTON_NAMES.index(button_name)
        value = self.message.buttons[index]
        self.message.buttons[index] = not value

    def handleKey(self, key):
        if key in self.handlers:
            self.handlers[key]()
        return key == 'z' or key == '\x03'

    def publish_message(self):
        self.publisher.publish(self.message)

    def readLoop(self):
        key = self.readKey()
        if self.handleKey(key):
            self.input_timer.cancel()
            self.publish_timer.cancel()
            self.destroy_node()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    try:
        rcemu = RCEmulator()
    except Exception as err:
        print('Error while starting RCEmulator node: ' + str(err))
        raise err
    else:
        # if there were no exceptions in running the section inside try, run the following
        # loop until the exit key is pressed
        while rclpy.ok() and not rcemu.exiting:
            rclpy.spin_once(rcemu)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
