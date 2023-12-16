import rclpy
from rclpy.node import Node
import yaml
from yaml.loader import SafeLoader
# from reseq_interfaces.msg import Remote
from geometry_msgs.msg import Twist

"""
ROS node that handles scaling of the remote controller data into physical variables used
by the motors

It receives a packet from the remote controller and rescales end_effector data 
(pitch, head_pitch, head_yaw) and the Twist data used by Agevar 
(linear velocity, angular velocity)
"""

class Scaler(Node):
    def __init__(self):
        super().__init__("scaler")
        # TODO: read file passed as ROS argument
        with open("src/reseq_ros2/reseq_ros2/config.yaml") as f:
            self.config = yaml.load(f, Loader=SafeLoader)
            print(self.config)

        self.create_subscription(
            None, #TODO: Name Message
            None, #TODO: Name topic ("/remote")
            self.remote_callback,
            10
        )

        self.enea_pub = self.create_publisher(
            None, #TODO: Name Message
            None, #TODO: Name topic: ("/end_effector_vel")
            10
        )

        self.agevar_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.get_logger().info("Scaler node started")

    def remote_callback(self, data): pass

    def agevarScaler(self, data): pass

    def endEffectorScaler(self, data): pass


def main(args=None):
    rclpy.init(args=args)
    try:
        scaler = Scaler()
    except Exception as err:
        print("Error while starting Scaler node: " + str(err))
        rclpy.shutdown()
    else:
        rclpy.spin(scaler)
        scaler.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
