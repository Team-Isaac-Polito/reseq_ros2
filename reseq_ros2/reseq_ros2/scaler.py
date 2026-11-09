from __future__ import annotations

import traceback
from collections.abc import Callable
from dataclasses import dataclass, replace
from enum import Enum, IntEnum
from typing import ClassVar

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Client, Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from reseq_interfaces.msg import Remote

"""
ROS node that handles scaling of the remote controller data into physical variables used
by the motors

It receives a packet from the remote controller and rescales end_effector data
(pitch, head_pitch, head_roll) and the Twist data used by traction controllers
(linear velocity, angular velocity)

It also handles the button presses and switches of the remote controller
using a system of handlers that call the appropriate service, given an optional condition
and an optional hook function to be executed after the service is called.

The node never talks to the CAN bus directly: hardware actions (beak, LED, joint torque)
are published as ros2_control controller commands and encoded by reseq_hardware
(see reseq_hardware/config/mappings.yaml).
"""


class ControlMode(Enum):
    FTL = 0
    PIVOT = 1


class ButtonEnum(IntEnum):
    S1 = 0
    S2 = 1
    S3 = 2
    S4 = 3
    S5 = 4
    BGREEN = 5
    BBLACK = 6
    BRED = 7
    BWHITE = 8
    BBLUE = 9


# Switch controlling the inter-module joint torque of each module
TORQUE_SWITCHES: dict[int, ButtonEnum] = {2: ButtonEnum.S2, 3: ButtonEnum.S3}


@dataclass
class Handler:
    name: str
    button: ButtonEnum
    service: str | None
    action_type: str | None = None
    hook: Callable[[Scaler], None] | None = None
    inverted: bool | None = None
    condition: Callable[[list[bool]], bool] | None = None
    client: Client | None = None  # service client, created per node instance


class Scaler(Node):
    # OBSERVATIONS: The switches are zero in the upwards position,
    #               The buttons are zero when pressed
    handlers: ClassVar[list[Handler]] = [  # {button, service, inverted, condition, hook}
        Handler(
            name='Enable/Disable FTL Controller',
            button=ButtonEnum.BBLUE,
            service='/ftl_controller/enable',
            inverted=False,
            hook=lambda self: setattr(self, 'control_mode', ControlMode.FTL),
        ),
        Handler(
            name='Enable/Disable Pivot',
            button=ButtonEnum.BBLUE,
            service='/pivot_controller/enable',
            inverted=True,
            hook=lambda self: setattr(self, 'control_mode', ControlMode.PIVOT),
        ),
        Handler(
            name='Enable/Disable Pivot on Head',
            button=ButtonEnum.S5,
            service='/pivot_controller/pivot_on_head',
            inverted=True,
            condition=lambda b: not b[ButtonEnum.BBLUE],
        ),
        Handler(
            name='Switch MK2 Arm End-Effector Mode',
            button=ButtonEnum.S4,
            service='/cartesian_arm_controller/switch_vel',
            # Switches read False in the upper position. The arm controller
            # expects True=linear, False=rotation.
            inverted=True,
        ),
        Handler(
            name='Home MK2 Arm',
            button=ButtonEnum.BGREEN,
            service='/cartesian_arm_controller/go_home',
            inverted=False,
            action_type='trigger',
        ),
        Handler(
            name='Open/Close MK2 Arm Beak',
            button=ButtonEnum.BBLACK,
            service=None,
            inverted=False,
            action_type='beak',
        ),
        Handler(
            name='Toggle Module 1 LED',
            button=ButtonEnum.BWHITE,
            service=None,
            inverted=False,
            action_type='led',
        ),
        Handler(
            name='Autonomy Enable/Disable',
            button=ButtonEnum.BRED,
            service='/autonomy/enable',
            inverted=False,
            action_type='autonomy',
        ),
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
        self.control_mode = ControlMode.FTL
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

        # Handlers are copied per instance so the class-level definitions stay untouched
        self.handlers = [replace(h) for h in Scaler.handlers]
        for h in self.handlers:
            if h.service is None:
                continue  # handler publishes a controller command instead of calling a service
            srv_type = Trigger if h.action_type == 'trigger' else SetBool
            h.client = self.create_client(srv_type, h.service)

        self.create_subscription(Remote, '/remote', self.remote_callback, self.qos)

        self.arm_vel_pub = self.create_publisher(Vector3, arm_vel_topic, 10)

        # Joint lift velocity publisher
        self.lift_pub = self.create_publisher(Vector3, '/inter_module_lift_vel', 10)

        # Hardware actions, encoded to CAN by reseq_hardware (mappings.yaml):
        #   beak_controller   → mod1__beak/position        → 0x5B ARM_ROLL_6_SETPOINT
        #   led_controller    → mod1__led/brightness       → 0x75 LED_HP_BRIGHTNESS
        #   torque_controller → mod<N>__torque/joint (N≥2) → 0x74 TORQUE_ENABLE_DISABLE bits 2-3
        self.beak_pub = self.create_publisher(Float64MultiArray, '/beak_controller/commands', 10)
        self.beak_open_position = self.declare_parameter('beak_open_position', 1.0).value
        self.beak_closed_position = self.declare_parameter('beak_closed_position', 0.0).value

        self.led_pub = self.create_publisher(Float64MultiArray, '/led_controller/commands', 10)
        self.led_on_brightness = int(self.declare_parameter('led_on_brightness', 125).value)
        if not 0 <= self.led_on_brightness <= 255:
            raise ValueError(
                f'led_on_brightness must be in [0, 255], got {self.led_on_brightness}'
            )

        self.torque_pub = self.create_publisher(
            Float64MultiArray, '/torque_controller/commands', 10
        )
        # Must list the modules in the same order as torque_controller's joints
        self.torque_modules = list(self.declare_parameter('torque_modules', [2, 3]).value)
        unknown = [m for m in self.torque_modules if m not in TORQUE_SWITCHES]
        if unknown:
            raise ValueError(
                f'torque_modules {unknown} have no remote switch (supported: '
                f'{sorted(TORQUE_SWITCHES)})'
            )

        # Track previous torque switch states (None = nothing sent yet)
        self.prev_torque_switches: list[bool] | None = None
        # Track LED toggle state
        self.led_state = False
        # Track beak toggle state
        self.beak_state = False

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
        buttons = list(buttons)
        if buttons == self.previous_buttons:
            return

        for handler in self.handlers:
            if buttons[handler.button] == self.previous_buttons[handler.button] or not (
                handler.condition is None or handler.condition(buttons)
            ):
                continue
            # Momentary buttons read False while pressed: a press is a True -> False edge
            pressed = self.previous_buttons[handler.button] and not buttons[handler.button]

            if handler.action_type == 'trigger':
                if pressed:
                    handler.client.call_async(Trigger.Request())
                    self.get_logger().debug(f"Called service '{handler.name}'")
            elif handler.action_type == 'led':
                if pressed:
                    self.led_state = not self.led_state
                    brightness = self.led_on_brightness if self.led_state else 0
                    self.led_pub.publish(Float64MultiArray(data=[float(brightness)]))
                    self.get_logger().debug(f'LED brightness set to {brightness}')
            elif handler.action_type == 'beak':
                if pressed:
                    self.beak_state = not self.beak_state
                    beak_position = (
                        self.beak_open_position if self.beak_state else self.beak_closed_position
                    )
                    self.beak_pub.publish(Float64MultiArray(data=[float(beak_position)]))
                    self.get_logger().debug(f'Beak {"open" if self.beak_state else "close"}')
            elif handler.action_type == 'autonomy':
                if pressed:
                    next_state = not self.autonomy_enabled
                    handler.client.call_async(SetBool.Request(data=next_state))
                    self.get_logger().debug(
                        f'Autonomy {"enabled" if next_state else "disabled"} via button press'
                    )
            else:
                data = bool(handler.inverted) ^ bool(buttons[handler.button])
                handler.client.call_async(SetBool.Request(data=data))
                if handler.hook is not None and data:
                    handler.hook(self)
                self.get_logger().debug(
                    f"Called service '{handler.name}' for "
                    f'{handler.button.name}={buttons[handler.button]}, value={data}'
                )
        self.previous_buttons = buttons

    def handle_torque_switches(self, buttons: list[bool]):
        """Enable the inter-module joint motors torque while the module's switch is down.

        Switch down (True) → torque enabled, switch up (False) → joint motors passive.
        Only the joint group of mod<N>__torque is commanded: traction and roll torque are
        left to their hardware defaults.
        """
        switches = [bool(buttons[TORQUE_SWITCHES[m]]) for m in self.torque_modules]
        if switches == self.prev_torque_switches:
            return
        for i, (module, state) in enumerate(zip(self.torque_modules, switches)):
            if self.prev_torque_switches is None or state != self.prev_torque_switches[i]:
                self.get_logger().info(
                    f'{TORQUE_SWITCHES[module].name} switch changed: joint torque '
                    f'{"enabled" if state else "disabled"} for module {module}'
                )
        self.torque_pub.publish(Float64MultiArray(data=[float(s) for s in switches]))
        self.prev_torque_switches = switches

    def remote_callback(self, data: Remote):
        self.handle_buttons(data.buttons)

        cmd_vel = Twist()
        cmd_vel.linear.x = data.right.y  # Linear velocity (-1:1)
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
        s2 = data.buttons[ButtonEnum.S2]
        s3 = data.buttons[ButtonEnum.S3]

        # Torque of the inter-module joints follows the S2/S3 switches
        self.handle_torque_switches(data.buttons)

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

        if self.control_mode == ControlMode.FTL:
            cmd_vel = self.ftlScaler(cmd_vel)
        else:
            cmd_vel = self.pivotScaler(cmd_vel)
        self.speed_pub.publish(cmd_vel)

    def pivotScaler(self, data: Twist):
        data.linear.x = 0.0
        data.angular.z = self.scale(data.angular.z, self.r_angular_vel)
        return data

    def ftlScaler(self, data: Twist):
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
            f'Error in the Scaler node: {err!s}\n{traceback.format_exc()}'
        )
    else:
        scaler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
