import can
import rclpy
import reseq_ros2.constants as rc
import struct
import yaml
from can import Message
from geometry_msgs.msg import Twist
from rclpy.node import Node
from reseq_interfaces.msg import Motors
from std_msgs.msg import Float32  # deprecated?
from yaml.loader import SafeLoader


class Agevar(Node):
    def __init__(self):
        super().__init__("communication")
        # TODO: read file passed as ROS argument
        with open("src/reseq_ros2/reseq_ros2/config.yaml") as f:
            self.config = yaml.load(f, Loader=SafeLoader)
            print(self.config)

        # subscribe to remote (parsed by teleop_twist_joy)
        remote_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.remote_callback,
            10,
        )

        self.joint_subs = []
        self.motors_pubs = []
        for i in range(len(self.config["modules"])):
            info = self.config["modules"][i]

            # subscribe to feedback from joints
            if info['hasJoint']:
                s = self.create_subscription(
                    Float32,
                    f"reseq/module{info['address']}/joint/yaw/feedback",
                    lambda msg: self.robot_feedback_callback(
                        msg, info["address"]),
                    10,
                )
                self.joint_subs.append(s)

            # create publisher for the motor topics
            p = self.create_publisher(
                Motors,
                f"reseq/module{info['address']}/motor/setpoint",
                10
            )
            self.motors_pubs.append(p)

    def remote_callback(self, msg):
        self.get_logger().info(f"Twist x:{msg.linear.x}, y:{msg.linear.x}")

    def robot_feedback_callback(self, msg, module_num):
        pass


def main(args=None):
    rclpy.init(args=args)
    try:
        agevar = Agevar()
    except Exception as err:
        print("Error while starting Agevar node: " + str(err))
        rclpy.shutdown()
    else:
        rclpy.spin(agevar)
        agevar.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
