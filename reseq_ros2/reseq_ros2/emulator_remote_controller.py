import rclpy
from reseq_interfaces.msg import Remote
from rclpy.node import Node
from pynput import keyboard

class EmulatorRemoteController(Node):
    def __init__(self):
        super().__init__("emulator_remote_controller")
        # publish to topic "/remote"
        self.publisher = self.create_publisher(Remote, '/remote', 10)
        self.get_logger().info("EmulatorRemoteController node started")
        # create timer to start the function
        self.timer = self.create_timer(0.1, self.readLoop)
        self.exit = False
        # values should be already automatically be set to 0
        self.previousKey = ' ' # initialise to a random value
        self.previousValue = 0
        self.operation_sign = {"+": 1, "-":-1}
    
    def readLoop(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as self.listener:
            self.listener.join()  # Wait for the listener thread to terminate

    def on_press(self, key):
        """Callback for key press events."""
        message = Remote()
        try:
            # make lower case
            key.char = key.char.lower()
            # Start end effector commands
            if key.char == 'w':
                # print("forward")
                if (key.char == self.previousKey):
                    # increase of 0.1 (if previous value was 0), otherwise increase 10%
                    # if same key, it means value is not 0
                    message.left.y = min(1,self.previousValue * 1.1)
                    self.previousValue = min(1,self.previousValue * 1.1)
                else:
                    # not the same key, so previous value has to 0
                    self.previousValue = 0
                    message.left.y = 0.1
                    self.previousValue = 0.1
                    self.previousKey = key.char
                self.publisher.publish(message)
            elif key.char == 's':
                # print("backward")
                if (key.char == self.previousKey):
                    # increase of 0.1 (if previous value was 0), otherwise increase 10%
                    # if same key, it means value is not 0
                    message.left.y = -min(1,-self.previousValue * 1.1)
                    self.previousValue = -min(1,-self.previousValue * 1.1)
                else:
                    # not the same key, so previous value has to 0
                    self.previousValue = 0
                    message.left.y = -0.1
                    self.previousValue = -0.1
                    self.previousKey = key.char
                self.publisher.publish(message)
            elif key.char == 'a':
                # print("left")
                if (key.char == self.previousKey):
                    # increase of 0.1 (if previous value was 0), otherwise increase 10%
                    # if same key, it means value is not 0
                    message.left.x = -min(1,-self.previousValue * 1.1)
                    self.previousValue = -min(1,-self.previousValue * 1.1)
                else:
                    # not the same key, so previous value has to 0
                    self.previousValue = 0
                    message.left.x = -0.1
                    self.previousValue = -0.1
                    self.previousKey = key.char
                self.publisher.publish(message)
            elif key.char == 'd':
                # print("right")
                if (key.char == self.previousKey):
                    # increase of 0.1 (if previous value was 0), otherwise increase 10%
                    # if same key, it means value is not 0
                    message.left.x = min(1,self.previousValue * 1.1)
                    self.previousValue = min(1,self.previousValue * 1.1)
                else:
                    # not the same key, so previous value has to 0
                    self.previousValue = 0
                    message.left.x = 0.1
                    self.previousValue = 0.1
                    self.previousKey = key.char
                self.publisher.publish(message)
            elif key.char == 'q':
                # print("CCW")
                if (key.char == self.previousKey):
                    # increase of 0.1 (if previous value was 0), otherwise increase 10%
                    # if same key, it means value is not 0
                    message.left.z = -min(1,-self.previousValue * 1.1)
                    self.previousValue = -min(1,-self.previousValue * 1.1)
                else:
                    # not the same key, so previous value has to 0
                    self.previousValue = 0
                    message.left.z = -0.1
                    self.previousValue = -0.1
                    self.previousKey = key.char
                self.publisher.publish(message)
            elif key.char == 'e':
                # print("CW")
                if (key.char == self.previousKey):
                    # increase of 0.1 (if previous value was 0), otherwise increase 10%
                    # if same key, it means value is not 0
                    message.left.z = min(1,self.previousValue * 1.1)
                    self.previousValue = -min(1,self.previousValue * 1.1)
                else:
                    # not the same key, so previous value has to 0
                    self.previousValue = 0
                    message.left.z = 0.1
                    self.previousValue = 0.1
                    self.previousKey = key.char
                self.publisher.publish(message)
            # Start cmd_vel commands
            elif key.char == 'i':
                # print("forward")
                # if the key is equal to the previous one, increase 10% of the previous value, max 1
                if (key.char == self.previousKey):
                    # increase of 0.1 (if previous value was 0), otherwise increase 10%
                    # if same key, it means value is not 0
                    message.right.y = min(1,self.previousValue * 1.1)
                    self.previousValue = min(1,self.previousValue * 1.1)
                else:
                    # not the same key, so previous value has to 0
                    self.previousValue = 0
                    message.right.y = 0.1
                    self.previousValue = 0.1
                    self.previousKey = key.char
                self.publisher.publish(message)
            elif key.char == 'k':
                # print("backward")
                if (key.char == self.previousKey):
                    # increase of 0.1 (if previous value was 0), otherwise increase 10%
                    # if same key, it means value is not 0
                    message.right.y = -min(1,-self.previousValue * 1.1)
                    self.previousValue = -min(1,-self.previousValue * 1.1)
                else:
                    # not the same key, so previous value has to 0
                    self.previousValue = 0
                    message.right.y = -0.1
                    self.previousValue = -0.1
                    self.previousKey = key.char
                self.publisher.publish(message)
            elif key.char == 'l':
                # print("right")
                if (key.char == self.previousKey):
                    # increase of 0.1 (if previous value was 0), otherwise increase 10%
                    # if same key, it means value is not 0
                    message.right.x = min(1,self.previousValue * 1.1)
                    self.previousValue = min(1,self.previousValue * 1.1)
                else:
                    # not the same key, so previous value has to 0
                    self.previousValue = 0
                    message.right.x = 0.1
                    self.previousValue = 0.1
                    self.previousKey = key.char
                self.publisher.publish(message)
            elif key.char == 'j':
                # print("left")
                if (key.char == self.previousKey):
                    # increase of 0.1 (if previous value was 0), otherwise increase 10%
                    # if same key, it means value is not 0
                    message.right.x = -min(1,-self.previousValue * 1.1)
                    self.previousValue = -min(1,-self.previousValue * 1.1)
                else:
                    # not the same key, so previous value has to 0
                    self.previousValue = 0
                    message.right.x = -0.1
                    self.previousValue = -0.1
                    self.previousKey = key.char
                self.publisher.publish(message)
            # if b pressed, make cmd_vel movements faser
            elif key.char == 'b':
                print("fast")
                # it works only if previous key is I,K,J,L
                if self.previousKey in ['i','k']:
                    if self.previousKey == 'i':
                        message.right.y = min(1,self.previousValue * 1.5)
                        self.previousValue = min(1,self.previousValue * 1.5)
                    else:
                        message.right.y = -min(1,-self.previousValue * 1.5)
                        self.previousValue = -min(1,-self.previousValue * 1.5)
                    self.publisher.publish(message)
                elif self.previousKey in ['j','l']:
                    if self.previousKey == 'j':
                        message.right.x = -min(1,-self.previousValue * 1.5)
                        self.previousValue = -min(1,-self.previousValue * 1.5)
                    else:
                        message.right.x = min(1,self.previousValue * 1.5)
                        self.previousValue = min(1,self.previousValue * 1.5)
                    self.publisher.publish(message)
            elif key.char == 'z':
                # stop the listener, otherwise it will keep listening for keys
                self.listener.stop()
                # set the flag to true
                self.exit = True
            print(f"pressed {key.char}")
        except AttributeError:
            pass

    def on_release(self, key):
        """Callback for key release events."""
        try:
            pass
        except AttributeError:
            pass

def main(args = None):
    rclpy.init(args = args)
    try:
        emulator_remote_controller = EmulatorRemoteController()
        # spin once, each time check if listener stopped
        while rclpy.ok() and not emulator_remote_controller.exit:
            rclpy.spin_once(emulator_remote_controller)
    except Exception as err:
        print("Error while starting EmulatorRemoteController node: " + str(err))
        rclpy.shutdown()
    finally:
        emulator_remote_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()