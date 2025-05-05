import traceback
from enum import Enum, IntEnum

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import SetBool

from reseq_interfaces.msg import EndEffector, Remote

"""
ROS node that handles scaling of the remote controller data into physical variables used
by the motors

It receives a packet from the remote controller and rescales end_effector data
(pitch, head_pitch, head_roll) and the Twist data used by Agevar
(linear velocity, angular velocity)

It also handles the button presses and switches of the remote controller
using a system of handlers that call the appropriate service, given an optional condition
and an optional hook function to be executed after the service is called.
"""


class Scaler(Node):
    control_mode_enum: Enum = Enum('ControlMode', 'AGEVAR, PIVOT')
    buttons_enum: IntEnum = IntEnum(
        'Buttons', 'S1, S2, S3, S4, S5 BGREEN, BBLACK, BRED, BWHITE, BBLUE', start=0
    )

    # OBSERVATIONS: The switches are zero in the upwards position,
    #               The buttons are zero when pressed
    handlers: list[dict] = [  # {button, service, inverted, condition, hook}
        {
            'name': 'Enable/Disable Agevar',
            'button': buttons_enum.BBLUE,
            'service': '/agevar/enable',
            'inverted': False,
            'hook': lambda self: setattr(self, 'control_mode', Scaler.control_mode_enum.AGEVAR),
        },
        {
            'name': 'Enable/Disable Pivot',
            'button': buttons_enum.BBLUE,
            'service': '/pivot_controller/enable',
            'inverted': True,
            'hook': lambda self: setattr(self, 'control_mode', Scaler.control_mode_enum.PIVOT),
        },
        {
            'name': 'Enable/Disable Pivot on Head',
            'button': buttons_enum.S5,
            'service': '/pivot_controller/pivot_on_head',
            'inverted': True,
            'condition': lambda b: not b[Scaler.buttons_enum.BBLUE],
        },
    ]

    def __init__(self):
        super().__init__('scaler')
        # initialize the button/switch handlers
        self.previous_buttons = [False, False, False, False, False, True, True, True, True, True]
        self.control_mode = Scaler.control_mode_enum.AGEVAR
        self.enea_enabled = False

        # Declaring parameters and getting values
        version = self.declare_parameter('version', 'mk1').get_parameter_value().string_value

        self.r_linear_vel = (
            self.declare_parameter('r_linear_vel', [0.0]).get_parameter_value().double_array_value
        )
        self.r_inverse_radius = (
            self.declare_parameter('r_inverse_radius', [0.0])
            .get_parameter_value()
            .double_array_value
        )
        self.r_angular_vel = (
            self.declare_parameter('r_angular_vel', [0.0]).get_parameter_value().double_array_value
        )

        if version == 'mk1':
            self.enea_enabled = True
            self.r_pitch_vel = (
                self.declare_parameter('r_pitch_vel', [0])
                .get_parameter_value()
                .integer_array_value
            )
            self.r_head_pitch_vel = (
                self.declare_parameter('r_head_pitch_vel', [0])
                .get_parameter_value()
                .integer_array_value
            )
            self.r_head_roll_vel = (
                self.declare_parameter('r_head_roll_vel', [0])
                .get_parameter_value()
                .integer_array_value
            )

        for h in self.handlers:
            h['service'] = self.create_client(SetBool, h['service'])

        self.create_subscription(Remote, '/remote', self.remote_callback, 10)

        if self.enea_enabled:
            self.enea_pub = self.create_publisher(EndEffector, '/end_effector', 10)

        self.speed_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Scaler node started')

    def handle_buttons(self, buttons: list[bool]):
        if buttons == self.previous_buttons:
            return

        for handler in self.handlers:
            if buttons[handler['button']] != self.previous_buttons[handler['button']]:
                if 'condition' not in handler or handler['condition'](buttons):
                    data = handler['inverted'] ^ buttons[handler['button']]
                    handler['service'].call_async(SetBool.Request(data=data))
                    if 'hook' in handler:
                        handler['hook'](self)
                    self.get_logger().debug(
                        f"Called service '{handler['name']}' for {handler['button'].name}={buttons[handler['button']]}, value={data}"  # noqa
                    )
        self.previous_buttons = buttons

    def remote_callback(self, data: Remote):
        self.handle_buttons(data.buttons)

        cmd_vel = Twist()
        cmd_vel.linear.x = data.right.y  # Linear velocity (-1:1)
        # inverse of Radius of curvature (AGEVAR) or angular velocity (PIVOT) (-1:1)
        cmd_vel.angular.z = -data.right.x

        if self.control_mode == Scaler.control_mode_enum.AGEVAR:
            cmd_vel = self.agevarScaler(cmd_vel)
        else:
            cmd_vel = self.pivotScaler(cmd_vel)
        self.speed_pub.publish(cmd_vel)

        if self.enea_enabled:
            end_e = EndEffector()
            end_e.pitch_vel = data.left.y
            end_e.head_pitch_vel = data.left.z
            end_e.head_roll_vel = data.left.x

            end_e = self.endEffectorScaler(end_e)
            self.enea_pub.publish(end_e)

    def pivotScaler(self, data: Twist):
        data.linear.x = 0.0
        data.angular.z = self.scale(data.angular.z, self.r_angular_vel)
        return data

    def agevarScaler(self, data: Twist):
        data.linear.x = self.scale(data.linear.x, self.r_linear_vel)
        data.angular.z = self.scale(data.angular.z, self.r_inverse_radius)
        data.angular.z *= data.linear.x  # Angular vel
        return data

    def endEffectorScaler(self, data: EndEffector):
        data.pitch_vel = self.scale(data.pitch_vel, self.r_pitch_vel)
        data.head_pitch_vel = self.scale(data.head_pitch_vel, self.r_head_pitch_vel)
        data.head_roll_vel = self.scale(data.head_roll_vel, self.r_head_roll_vel)
        return data

    def scale(self, val, scaling_range):
        return (val + 1) / 2 * (scaling_range[1] - scaling_range[0]) + scaling_range[0]


def main(args=None):
    rclpy.init(args=args)
    try:
        scaler = Scaler()
        rclpy.spin(scaler)
    except Exception as err:
        rclpy.logging.get_logger('scaler').fatal(
            f'Error in the Scaler node: {str(err)}\n{traceback.format_exc()}'
        )
        raise err
    else:
        scaler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
