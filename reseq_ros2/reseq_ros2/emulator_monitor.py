import rclpy
from rclpy.node import Node
from reseq_interfaces.msg import Remote

class EmulatorMonitor(Node):
    def __init__(self):
        super().__init__("emulator_monitor")
        self.subscription = self.create_subscription(Remote,'/remote',self.monitor_callback,10)
        self.get_logger().info("EmulatorMonitor node started")
        self.msg = None

    def monitor_callback(self, msg):
        # self.get_logger().info(f"Received: {msg}")
        self.msg = msg

def main(args = None):
    rclpy.init(args = args)
    try:
        emulator_monitor = EmulatorMonitor()
        rclpy.spin(emulator_monitor)
    except Exception as err:
        print("Error while starting EmulatorMonitor node: " + str(err))
        rclpy.shutdown()
    else:
        emulator_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()