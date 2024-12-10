import rclpy
from reseq_interfaces.msg import Remote
from rclpy.node import Node
import sys, select, termios, tty

# retrieve settings from file descriptor (stdin)
settings = termios.tcgetattr(sys.stdin)

class EmulatorRemoteController(Node):
    def __init__(self):
        super().__init__("emulator_remote_controller")
        # create teleop_twist_keyboard node
        self.emulator = rclpy.create_node("teleop_twist_keyboard")
        self.publisher = self.create_publisher(Remote, '/remote',10)
        self.get_logger().info("EmulatorRemoteController node started")
        # create timer to start the function
        self.timer = self.create_timer(0.08, self.readLoop)
        self.has_to_exit = False
        # values should be already automatically be set to 0
        self.previousKey = ' ' # initialise to a random value
        self.previousValue = 0.0
        self.increment = 0.1
        # self.states = {"Normal": self.normal_value, "Double": self.normal_value*2, "Half": self.normal_value/2}
        self.defaultMessage = Remote()
        self.defaultMessage.right.x = 0.0
        self.defaultMessage.right.y = 0.0
        self.defaultMessage.right.z = 0.0
        self.defaultMessage.left.x = 0.0
        self.defaultMessage.left.y = 0.0
        self.defaultMessage.left.z = 0.0

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
    
    def handleKey(self, key, message = Remote()):
        # make lower case
        key = key.lower()
        if key == 'w':
            # print("forward")
            if (key == self.previousKey):
                message.left.y = min(1.0,self.previousValue * (self.increment+1))
                self.previousValue = min(1.0,self.previousValue * (self.increment+1))
            else:
                message.left.y = self.increment
                self.previousValue = self.increment
                self.previousKey = key
            self.publisher.publish(message)
        elif key == 's':
            # print("backward")
            if (key == self.previousKey):
                message.left.y = -min(1.0,-self.previousValue * (self.increment+1))
                self.previousValue = -min(1.0,-self.previousValue * (self.increment+1))
            else:
                message.left.y = -self.increment
                self.previousValue = -self.increment
                self.previousKey = key
            self.publisher.publish(message)
        elif key == 'a':
            # print("left")
            if (key == self.previousKey):
                message.left.x = -min(1.0,-self.previousValue * (self.increment+1))
                self.previousValue = -min(1.0,-self.previousValue * (self.increment+1))
            else:
                message.left.x = -self.increment
                self.previousValue = -self.increment
                self.previousKey = key
            self.publisher.publish(message)
        elif key == 'd':
            # print("right")
            if (key == self.previousKey):
                message.left.x = min(1.0,self.previousValue * (self.increment+1))
                self.previousValue = min(1.0,self.previousValue * (self.increment+1))
            else:
                message.left.x = self.increment
                self.previousValue = self.increment
                self.previousKey = key
            self.publisher.publish(message)
        elif key == 'q':
            # print("CCW")
            if (key == self.previousKey):
                message.left.z = -min(1.0,-self.previousValue * (self.increment+1))
                self.previousValue = -min(1.0,-self.previousValue * (self.increment+1))
            else:
                message.left.z = -self.increment
                self.previousValue = -self.increment
                self.previousKey = key
            self.publisher.publish(message)
        elif key == 'e':
            # print("CW")
            if (key == self.previousKey):
                message.left.z = min(1.0,self.previousValue * (self.increment+1))
                self.previousValue = min(1.0,self.previousValue * (self.increment+1))
            else:
                message.left.z = self.increment
                self.previousValue = self.increment
                self.previousKey = key
            self.publisher.publish(message)
        # Start cmd_vel commands
        elif key == 'i':
            # print("forward")
            if (key == self.previousKey):
                message.right.y = min(1.0,self.previousValue * (self.increment+1))
                self.previousValue = min(1.0,self.previousValue * (self.increment+1))
            else:
                message.right.y = self.increment
                self.previousValue = self.increment
                self.previousKey = key
            self.publisher.publish(message)
        elif key == 'k':
            # print("backward")
            if (key == self.previousKey):
                message.right.y = -min(1.0,-self.previousValue * (self.increment+1))
                self.previousValue = -min(1.0,-self.previousValue * (self.increment+1))
            else:
                message.right.y = -self.increment
                self.previousValue = -self.increment
                self.previousKey = key
            self.publisher.publish(message)
        elif key == 'l':
            # print("right")
            if (key == self.previousKey):
                message.right.x = min(1.0,self.previousValue * (self.increment+1))
                self.previousValue = min(1.0,self.previousValue * (self.increment+1))
            else:
                message.right.x = self.increment
                self.previousValue = self.increment
                self.previousKey = key
            self.publisher.publish(message)
        elif key == 'j':
            # print("left")
            if (key == self.previousKey):
                message.right.x = -min(1.0,-self.previousValue * (self.increment+1))
                self.previousValue = -min(1.0,-self.previousValue * (self.increment+1))
            else:
                message.right.x = -self.increment
                self.previousValue = -self.increment
                self.previousKey = key
            self.publisher.publish(message)
        # if b pressed, make cmd_vel movements faster
        elif key == 'b':
            self.increment /= 2
        elif key == 'h':
            self.increment *= 2
        elif key == 'z':
            raise Exception("Exit requested")
        else:
            # pass to default
            self.publisher.publish(self.defaultMessage)
        return self.has_to_exit

    def readLoop(self):
        message = Remote()
        key = self.readKey()
        if self.handleKey(key, message):
            # destroy node
            self.emulator.destroy_node()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args = None):
    rclpy.init(args = args)
    try:
        emulator_remote_controller = EmulatorRemoteController()
    except Exception as err:
        print("Error while starting EmulatorRemoteController node: " + str(err))
        raise err
    else:
        rclpy.spin(emulator_remote_controller)
        emulator_remote_controller.emulator.destroy_node()
        emulator_remote_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()