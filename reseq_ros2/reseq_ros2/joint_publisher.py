import rclpy
from rclpy.node import Node

class JointPublisher(Node):
    def __init__(self):
        super().__init__("joint_publisher")
        self.get_logger().info("JointPublisher node started")

def main(args = None):
    rclpy.init(args = args)
    try:
        joint_publisher = JointPublisher()
    except Exception as err:
        print("Error while starting JointPublisher node: " + str(err))
        rclpy.shutdown()
    else:
        rclpy.spin(joint_publisher)
        joint_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
