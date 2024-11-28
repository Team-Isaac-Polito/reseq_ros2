import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from pynput import keyboard

class EmulatorRemoteController(Node):
    def __init__(self):
        super().__init__("emulator_remote_controller")
        # publish to topic "/remote"
        self.publisher = self.create_publisher(Twist, '/remote', 10)
        self.get_logger().info("EmulatorRemoteController node started")
        # create timer to start the function
        self.timer = self.create_timer(0.1, self.readLoop)
        self.exit = False
    
    def readLoop(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as self.listener:
            self.listener.join()  # Wait for the listener thread to terminate

    def on_press(self, key):
        """Callback for key press events."""
        try:
            if key.char == 'w':
                print("forward")
            elif key.char == 's':
                print("backward")
            elif key.char == 'a':
                print("left")
            elif key.char == 'd':
                print("rigth")
            elif key.char == 'q':
                # stop the listener, otherwise it will keep listening for keys
                self.listener.stop()
                # set the flag to true
                self.exit = True
        except AttributeError:
            pass

    def on_release(self, key):
        """Callback for key release events."""
        try:
            if key.char in ['w', 's']:
                print("stop")
            elif key.char in ['a', 'd']:
                print("stop")
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