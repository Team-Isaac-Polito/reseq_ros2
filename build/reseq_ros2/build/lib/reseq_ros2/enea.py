import traceback
from enum import Enum
from time import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from reseq_interfaces.msg import EndEffector


class EE_Enum(Enum):
    PITCH = 0
    HEAD_PITCH = 1
    HEAD_ROLL = 2


class Enea(Node):
    def __init__(self):
        super().__init__('enea')
        # Declaring parameters and getting values
        self.pitch = self.declare_parameter('pitch', 0).get_parameter_value().integer_value
        self.head_pitch = (
            self.declare_parameter('head_pitch', 0).get_parameter_value().integer_value
        )
        self.head_roll = self.declare_parameter('head_roll', 0).get_parameter_value().integer_value
        self.servo_speed = (
            self.declare_parameter('servo_speed', 0).get_parameter_value().integer_value
        )
        self.r_pitch = (
            self.declare_parameter('r_pitch', [0]).get_parameter_value().integer_array_value
        )
        self.r_head_pitch = (
            self.declare_parameter('r_head_pitch', [0]).get_parameter_value().integer_array_value
        )
        self.r_head_roll = (
            self.declare_parameter('r_head_roll', [0]).get_parameter_value().integer_array_value
        )
        self.pitch_conv = (
            self.declare_parameter('pitch_conv', 0.0).get_parameter_value().double_value
        )
        self.end_effector = (
            self.declare_parameter('end_effector', 0).get_parameter_value().integer_value
        )

        self.create_subscription(
            EndEffector,
            '/end_effector',
            self.consume_velocities,
            10,
        )

        self.pubs = {}
        addr = self.end_effector
        self.pubs[EE_Enum.PITCH] = self.create_publisher(
            Int32,
            f'reseq/module{addr}/end_effector/pitch/setpoint',
            10,
        )
        self.pubs[EE_Enum.HEAD_PITCH] = self.create_publisher(
            Int32,
            f'reseq/module{addr}/end_effector/head_pitch/setpoint',
            10,
        )
        self.pubs[EE_Enum.HEAD_ROLL] = self.create_publisher(
            Int32,
            f'reseq/module{addr}/end_effector/head_roll/setpoint',
            10,
        )

        self.get_logger().info('Node EnEA started successfully')

        self.previous_time = -1
        self.post()
        self.get_logger().info('EnEA: Moving to start position')

    def consume_velocities(self, msg: EndEffector):
        t = time()
        dt = t - self.previous_time
        self.previous_time = t

        dp = (-1) * msg.pitch_vel * dt
        dp = self.constrain_delta_pitch(dp)

        self.pitch += dp
        self.head_roll += (-1) * msg.head_roll_vel * dt
        self.head_pitch += msg.head_pitch_vel * dt + self.pitch_conv * dp

        self.constrain()

        self.get_logger().debug(
            f'Output: pitch={self.pitch}, head_pitch={self.head_pitch}, head_roll={self.head_roll}'
        )

        self.post()

    def post(self):
        """Send the output positions to the communication node."""
        self.pubs[EE_Enum.PITCH].publish(Int32(data=int(self.pitch)))
        self.pubs[EE_Enum.HEAD_PITCH].publish(Int32(data=int(self.head_pitch)))
        self.pubs[EE_Enum.HEAD_ROLL].publish(Int32(data=int(self.head_roll)))

    def constrain(self):
        """
        Constrains the values of `pitch`, `head_pitch` and `head_roll` to be inside
        of the intervals defined in the constants file `r_...`

        Theoretically the pitch value is already constrained by the constraints of dp
        """
        if self.pitch < self.r_pitch[0]:
            self.pitch = self.r_pitch[0]
        if self.pitch > self.r_pitch[1]:
            self.pitch = self.r_pitch[1]

        if self.head_pitch < self.r_head_pitch[0]:
            self.head_pitch = self.r_head_pitch[0]
        if self.head_pitch > self.r_head_pitch[1]:
            self.head_pitch = self.r_head_pitch[1]

        if self.head_roll < self.r_head_roll[0]:
            self.head_roll = self.r_head_roll[0]
        if self.head_roll > self.r_head_roll[1]:
            self.head_roll = self.r_head_roll[1]

    def constrain_delta_pitch(self, dp):
        """
        Constrains the value of the pitch movement, it is used to properly sync
        the pitch and head_pitch movement if the arm reaches the boundaries but
        the head could still move
        """
        if self.pitch + dp > self.r_pitch[1]:
            dp = self.r_pitch[1] - self.pitch
        if self.pitch + dp < self.r_pitch[0]:
            dp = self.r_pitch[0] - self.pitch
        return dp


def main(args=None):
    rclpy.init(args=args)
    try:
        enea = Enea()
        rclpy.spin(enea)
    except Exception as err:
        rclpy.logging.get_logger('enea').fatal(
            f'Error in the Enea node: {str(err)}\n{traceback.format_exc()}'
        )
        raise err
    else:
        enea.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
