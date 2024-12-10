import rclpy
from reseq_interfaces.msg import Remote
from rclpy.node import Node

class EmulatorTrajectory(Node):
    def __init__(self):
        super().__init__("emulator_trajectory")
        self.publisher = self.create_publisher(Remote, '/remote',10)
        self.get_logger().info("EmulatorTrajectory node started")
        self.timer = self.create_timer(0.08, self.readLoop)
        self.circleMessage = Remote()
        self.circleMessage.right.x = 0.1
        self.circleMessage.right.y = 0.1
        self.circleMessage.right.z = 0.1
        self.circleMessage.left.x = 0.1
        self.circleMessage.left.y = 0.1
        self.circleMessage.left.z = 0.1
        self.stop_emulator_timer = self.create_timer(60.0, self.stopEmulator)
        self.has_to_exit = False

    def readLoop(self):
        self.publisher.publish(self.circleMessage)

    def stopEmulator(self):
        raise Exception("Exit requested")

def main(args = None):
    rclpy.init(args = args)
    try:
        emulator_trajectory = EmulatorTrajectory()
    except Exception as err:
        print("Error while starting EmulatorTrajectory node: " + str(err))
        raise err
    else:
        rclpy.spin(emulator_trajectory)
        emulator_trajectory.emulator.destroy_node()
        emulator_trajectory.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()