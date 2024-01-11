import rclpy
from rclpy.node import Node
from yaml.loader import SafeLoader
from reseq_interfaces.msg import EndEffector
import reseq_ros2.constants as rc
from std_msgs.msg import Int32
import yaml
from enum import Enum
from time import time

class EE_Enum(Enum):
    PITCH = 0
    HEAD_PITCH = 1
    HEAD_YAW = 2

class Enea(Node):


    def __init__(self):
        super().__init__("enea")
        # TODO: read file passed as ROS argument
        with open("src/reseq_ros2/reseq_ros2/config.yaml") as f:
            self.config = yaml.load(f, Loader=SafeLoader)
            print(self.config)

        self.create_subscription(
            EndEffector,
            "/end_effector",
            self.consume_velocities,
            10,
        )

        self.pubs = {}
        addr = next(map(lambda x: x["address"], filter(lambda x: x["hasEndEffector"], self.config["modules"])))
        self.pubs[EE_Enum.PITCH] = self.create_publisher(
            Int32,
            f"reseq/module{addr}/end_effector/pitch/setpoint",
            10,
        )
        self.pubs[EE_Enum.HEAD_PITCH] = self.create_publisher(
            Int32,
            f"reseq/module{addr}/end_effector/head_pitch/setpoint",
            10,
        )
        self.pubs[EE_Enum.HEAD_YAW] = self.create_publisher(
            Int32,
            f"reseq/module{addr}/end_effector/head_yaw/setpoint",
            10,
        )

        self.get_logger().info("Node EnEA started successfully")
        
        self.previous_time = -1
        self.pitch = rc.i_pitch
        self.head_pitch = rc.i_head_pitch
        self.head_yaw = rc.i_head_yaw

        self.post()
        self.get_logger().info("EnEA: Moving to start position")

    def consume_velocities(self, msg: EndEffector): 
        t = time()
        dt = t - self.previous_time
        self.previous_time = t

        self.pitch += msg.pitch_vel*dt
        self.head_yaw += (-1)*msg.head_yaw_vel*dt
        self.head_pitch += (msg.head_pitch_vel + rc.pitch_conv*msg.pitch_vel)*dt

        self.constrain()

        self.get_logger().info(f"Output: pitch={self.pitch}, head_pitch={self.head_pitch}, head_yaw={self.head_yaw}")

        self.post()

    def post(self):
        """
        Sends the output positions to the communication node
        """
        
        self.pubs[EE_Enum.PITCH].publish(Int32(data=self.pitch))
        self.pubs[EE_Enum.HEAD_PITCH].publish(Int32(data=self.head_pitch))
        self.pubs[EE_Enum.HEAD_YAW].publish(Int32(data=self.head_yaw))

    def constrain(self):
        """
        Constrains the values of `pitch`, `head_pitch` and `head_yaw` to be inside
        of the intervals defined in the constants file `r_...`
        """
        if self.pitch < rc.r_pitch[0]: self.pitch = rc.r_pitch[0]
        if self.pitch > rc.r_pitch[1]: self.pitch = rc.r_pitch[1]

        if self.head_pitch < rc.r_head_pitch[0]: self.head_pitch = rc.r_head_pitch[0]
        if self.head_pitch > rc.r_head_pitch[1]: self.head_pitch = rc.r_head_pitch[1]

        if self.head_yaw < rc.r_head_yaw[0]: self.head_yaw = rc.r_head_yaw[0]
        if self.head_yaw > rc.r_head_yaw[1]: self.head_yaw = rc.r_head_yaw[1]



        




def main(args=None):
    rclpy.init(args=args)
    try:
        enea = Enea()
    except Exception as err:
        print("Error while starting Enea node: " + str(err))
        rclpy.shutdown()
    else:
        rclpy.spin(enea)
        enea.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
