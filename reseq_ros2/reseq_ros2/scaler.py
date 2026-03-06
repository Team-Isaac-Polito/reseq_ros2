from __future__ import annotations

import traceback
from enum import Enum, IntEnum

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool

from reseq_interfaces.msg import Remote

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
        'Buttons', 'S1, S2, S3, S4, S5, BGREEN, BBLACK, BRED, BWHITE, BBLUE', start=0
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
        {
            'name': 'Switch Arm Linear/Angular',
            'button': buttons_enum.BRED,
            'service': '/moveit_controller/switch_vel',
            'inverted': False,
        },
        {
            'name': 'Open/Close Beak',
            'button': buttons_enum.BGREEN,
            'service': '/moveit_controller/close_beak',
            'inverted': False,
        },
        {
            'name': 'Home Arm',
            'button': buttons_enum.BWHITE,
            'service': '/moveit_controller/home',
            'inverted': False,
        },
    ]

    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    def __init__(self):
        super().__init__('scaler')
        # initialize the button/switch handlers
        self.previous_buttons = [False, False, False, False, False, True, True, True, True, True]
        self.control_mode = Scaler.control_mode_enum.AGEVAR

        # Toggle state for push buttons (indices >= 5).
        # Initial value = NOT inverted, so the first press produces the expected action.
        self._push_toggle: dict[str, bool] = {}
        for h in self.handlers:
            if h['button'] >= 5:
                self._push_toggle[h['name']] = not h['inverted']

        self.r_linear_vel = (
            self.declare_parameter('r_linear_vel', [-0.1600, -0.1600])
            .get_parameter_value()
            .double_array_value
        )
        self.r_inverse_radius = (
            self.declare_parameter('r_inverse_radius', [-2.5478, -2.5478])
            .get_parameter_value()
            .double_array_value
        )
        self.r_angular_vel = (
            self.declare_parameter('r_angular_vel', [-2.4912, -2.4912])
            .get_parameter_value()
            .double_array_value
        )

        for h in self.handlers:
            h['service'] = self.create_client(SetBool, h['service'])

        self.create_subscription(Remote, '/remote', self.remote_callback, self.qos)

        self.moveit_pub = self.create_publisher(Vector3, '/mk2_arm_vel', 10)

        self.speed_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Scaler node started')

    def handle_buttons(self, buttons: list[bool]):
        if buttons == self.previous_buttons:
            return

        for handler in self.handlers:
            btn_idx = handler['button']
            if buttons[btn_idx] != self.previous_buttons[btn_idx]:
                is_push = btn_idx >= 5
                is_press = not buttons[btn_idx]  # Push buttons are False when pressed

                # Push buttons: only fire on press edge (True→False).
                # Switches: fire on both edges (existing behavior).
                if is_push and not is_press:
                    continue

                if 'condition' not in handler or handler['condition'](buttons):
                    if is_push and handler['name'] in self._push_toggle:
                        # Toggle: flip state on each press
                        self._push_toggle[handler['name']] = not self._push_toggle[handler['name']]
                        data = self._push_toggle[handler['name']]
                    else:
                        # Original logic for switches
                        data = handler['inverted'] ^ buttons[btn_idx]

                    if handler['service'].service_is_ready():
                        handler['service'].call_async(SetBool.Request(data=data))
                        self.get_logger().info(
                            f"[BUTTON] '{handler['name']}' "
                            f'{handler["button"].name} pressed → data={data}'
                        )
                    else:
                        self.get_logger().warn(
                            f"[BUTTON] Service not ready for '{handler['name']}'"
                        )
                    if 'hook' in handler and data:
                        handler['hook'](self)
                else:
                    self.get_logger().info(
                        f"[BUTTON] '{handler['name']}' skipped (condition=False)"
                    )
        self.previous_buttons = list(buttons)

    def remote_callback(self, data: Remote):
        self.handle_buttons(data.buttons)

        cmd_vel = Twist()
        cmd_vel.linear.x = data.right.y  # Linear velocity (-1:1)
        # inverse of Radius of curvature (AGEVAR) or angular velocity (PIVOT) (-1:1)
        cmd_vel.angular.z = -data.right.x

        # TODO probably to merge with another version of scaler.py

        self.moveit_pub.publish(
            Vector3(
                x=data.left.x,
                y=data.left.y,
                z=data.left.z,
            )
        )

        if self.control_mode == Scaler.control_mode_enum.AGEVAR:
            cmd_vel = self.agevarScaler(cmd_vel)
        else:
            cmd_vel = self.pivotScaler(cmd_vel)
        self.speed_pub.publish(cmd_vel)

    def pivotScaler(self, data: Twist):
        data.linear.x = 0.0
        data.angular.z = self.scale(data.angular.z, self.r_angular_vel)
        return data

    def agevarScaler(self, data: Twist):
        data.linear.x = self.scale(data.linear.x, self.r_linear_vel)
        data.angular.z = self.scale(data.angular.z, self.r_inverse_radius)
        data.angular.z *= data.linear.x  # Angular vel
        return data

    def scale(self, val, scaling_range):
        return (val + 1) / 2 * (scaling_range[1] - scaling_range[0]) + scaling_range[0]


def main(args=None):
    rclpy.init(args=args)
    try:
        scaler = Scaler()
        rclpy.spin(scaler)
    except KeyboardInterrupt:
        rclpy.logging.get_logger('scaler').warn('Scaler node interrupted by user')
    except Exception as err:
        rclpy.logging.get_logger('scaler').fatal(
            f'Error in the Scaler node: {str(err)}\n{traceback.format_exc()}'
        )
    else:
        scaler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
