from __future__ import annotations

import traceback
from enum import Enum, IntEnum

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from reseq_interfaces.msg import Remote

import can
import struct
import os
import sys

# Add STM32LowLevel tools to path for CanSender
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'STM32LowLevel', 'tools', 'can_tester'))
try:
    from sender import CanSender
    from protocol import MsgType, ModuleAddress
except ImportError:
    CanSender = None

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
            'name': 'Switch MK2 Arm End-Effector Mode',
            'button': buttons_enum.S4,
            'service': '/cartesian_arm_controller/switch_vel',
            # Switches read False in the upper position. The arm controller
            # expects True=linear, False=rotation.
            'inverted': True,
        },
        {
            'name': 'Home MK2 Arm',
            'button': buttons_enum.BGREEN,
            'service': '/cartesian_arm_controller/go_home',
            'inverted': False,
            'type': 'trigger',
        },
        {
            'name': 'Open/Close MK2 Arm Beak',
            'button': buttons_enum.S1,
            'service': '/moveit_controller/close_beak',
            'inverted': False,
        },
        {
            'name': 'Toggle Module 1 LED',
            'button': buttons_enum.BWHITE,
            'inverted': False,
            'type': 'led',
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
        self.autonomy_enabled = False

        self.r_linear_vel = (
            self.declare_parameter('r_linear_vel', [-0.1600, -0.1600])
            .get_parameter_value()
            .double_array_value
        )
        self.r_inverse_radius = (
            self.declare_parameter('r_inverse_radius', [-2.5478, 2.5478])
            .get_parameter_value()
            .double_array_value
        )
        self.r_angular_vel = (
            self.declare_parameter('r_angular_vel', [-2.4912, 2.4912])
            .get_parameter_value()
            .double_array_value
        )
        self.arm_input_scale = (
            self.declare_parameter('arm_input_scale', 1.0).get_parameter_value().double_value
        )
        self.arm_input_deadzone = (
            self.declare_parameter('arm_input_deadzone', 0.08).get_parameter_value().double_value
        )
        arm_vel_topic = self.declare_parameter('arm_vel_topic', '/mk2_arm_vel').value

        for h in self.handlers:
            if 'service' not in h:
                continue  # handler uses a custom action (e.g. LED toggle) instead of a service call
            if h.get('type') == 'trigger':
                h['service'] = self.create_client(Trigger, h['service'])
            else:
                h['service'] = self.create_client(SetBool, h['service'])

        self.create_subscription(Remote, '/remote', self.remote_callback, self.qos)

        self.arm_vel_pub = self.create_publisher(Vector3, arm_vel_topic, 10)

        # CAN sender for LED control
        self.can_sender = None
        can_channel = self.declare_parameter('can_channel', 'can0').value
        can_interface = self.declare_parameter('can_interface', 'socketcan').value
        if CanSender is not None:
            try:
                canbus = can.interface.Bus(channel=can_channel, bustype=can_interface)
                self.can_sender = CanSender(canbus)
                self.get_logger().info(f'CAN sender initialized on {can_channel}')
            except Exception as e:
                self.get_logger().warn(f'Failed to initialize CAN sender: {e}')
        
        # Joint lift velocity publisher
        self.lift_pub = self.create_publisher(Vector3, '/inter_module_lift_vel', 10)

        self.speed_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.autonomy_pub = self.create_publisher(Bool, '/autonomy/enabled', 10)

        self.create_service(SetBool, '/autonomy/enable', self.handle_autonomy_enable)

        self.get_logger().info('Scaler node started')

    def handle_autonomy_enable(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self.autonomy_enabled = request.data
        self.autonomy_pub.publish(Bool(data=self.autonomy_enabled))
        response.success = True
        response.message = 'Autonomy enabled' if self.autonomy_enabled else 'Autonomy disabled'
        return response

    def handle_buttons(self, buttons: list[bool]):
        if buttons == self.previous_buttons:
            return

        for handler in self.handlers:
            self.get_logger().debug(str(handler['button']))
            if buttons[handler['button']] != self.previous_buttons[handler['button']]:
                if 'condition' not in handler or handler['condition'](buttons):
                    if handler.get('type') == 'trigger':
                        handler['service'].call_async(Trigger.Request())
                        self.get_logger().debug(
                            f"Called service '{handler['name']}' for {handler['button'].name}={buttons[handler['button']]}"  # noqa
                        )
                    elif handler.get('type') == 'led':
                        data = handler['inverted'] ^ buttons[handler['button']]
                        if self.can_sender is not None:
                            brightness = 125 if data else 0
                            self.can_sender.led_hp_brightness(brightness)
                            self.get_logger().debug(f'LED brightness set to {brightness}')
                    else:
                        data = handler['inverted'] ^ buttons[handler['button']]
                        handler['service'].call_async(SetBool.Request(data=data))
                        if 'hook' in handler and data:
                            handler['hook'](self)
                        self.get_logger().debug(
                            f"Called service '{handler['name']}' for {handler['button'].name}={buttons[handler['button']]}, value={data}"  # noqa
                        )
        self.previous_buttons = buttons

    def remote_callback(self, data: Remote):
        self.handle_buttons(data.buttons)

        cmd_vel = Twist()
        cmd_vel.linear.x = data.right.y  # Linear velocity (-1:1)
        # Positive angular.z = counter-clockwise (left turn) per ROS convention.
        # Joystick right (data.right.x > 0) → positive angular.z → left turn.
        cmd_vel.angular.z = data.right.x

        # The app joystick is screen-oriented: X is right/left, Y is forward/back.
        # The arm controller expects Cartesian commands in arm_base_link:
        # +X forward, +Y left, +Z up. Invert screen X so pushing right moves right.
        self.arm_vel_pub.publish(
            Vector3(
                x=self.scale_arm_input(data.left.y),
                y=self.scale_arm_input(-data.left.x),
                z=self.scale_arm_input(data.left.z),
            )
        )

        # LIFTING CONTROL: S2, S3 Switches + Right Joystick Z axis
        # Right Z axis controls the pitch (lift amount)
        s2 = data.buttons[self.buttons_enum.S2]
        s3 = data.buttons[self.buttons_enum.S3]

        if s2 or s3:
            lift_msg = Vector3()
            lift_msg.x = data.right.z  # Pitch of the lifting module (from right joystick Z)
            lift_msg.y = 0.0

            if s2:
                # S2: Lift module 1
                lift_msg.z = 1.0  # Front lift type
                self.lift_pub.publish(lift_msg)

            if s3:
                # S3: Lift module 2
                lift_msg.z = 2.0
                self.lift_pub.publish(lift_msg)

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
        linear_input = data.linear.x
        angular_input = data.angular.z

        data.linear.x = self.scale(linear_input, self.r_linear_vel)
        if abs(linear_input) <= 0.08 and abs(angular_input) > 0.08:
            data.linear.x = 0.0
            data.angular.z = self.scale(angular_input, self.r_angular_vel)
        else:
            data.angular.z = self.scale(angular_input, self.r_inverse_radius)
            data.angular.z *= data.linear.x  # Angular vel
        return data

    def scale(self, val, scaling_range):
        return (val + 1) / 2 * (scaling_range[1] - scaling_range[0]) + scaling_range[0]

    def scale_arm_input(self, val: float) -> float:
        value = float(val)
        magnitude = abs(value)
        if magnitude <= self.arm_input_deadzone:
            return 0.0

        span = max(1.0 - self.arm_input_deadzone, 1e-6)
        scaled = ((magnitude - self.arm_input_deadzone) / span) * self.arm_input_scale
        scaled = max(-1.0, min(1.0, scaled))
        return scaled if value >= 0.0 else -scaled


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
