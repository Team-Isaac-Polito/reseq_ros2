from __future__ import annotations

import traceback
import warnings
from enum import Enum, IntEnum
import struct
import time

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

# Suppress pkg_resources deprecation warning from python-can library
warnings.filterwarnings("ignore", category=UserWarning, module="can.interfaces")


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
            'button': buttons_enum.BBLACK,
            'inverted': False,
            'type': 'beak',
        },
        {
            'name': 'Toggle Module 1 LED',
            'button': buttons_enum.BWHITE,
            'inverted': False,
            'type': 'led',
        },
        {
            'name': 'Autonomy Enable/Disable',
            'button': buttons_enum.BRED,
            'service': '/autonomy/enable',
            'inverted': False,
            'type': 'autonomy',
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

        # CAN sender for LED control and torque commands
        self.can_bus = None
        can_channel = self.declare_parameter('can_channel', 'can0').value
        can_interface = self.declare_parameter('can_interface', 'socketcan').value
        try:
            self.can_bus = can.interface.Bus(channel=can_channel, bustype=can_interface)
            self.get_logger().info(f'CAN bus initialized on {can_channel}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CAN bus on {can_channel}: {type(e).__name__}: {e}')
        
        # Joint lift velocity publisher
        self.lift_pub = self.create_publisher(Vector3, '/inter_module_lift_vel', 10)

        # Track previous S2/S3 switch states for torque control
        self.prev_s2 = None
        self.prev_s3 = None
        # Track LED toggle state
        self.led_state = False
        # Track beak toggle state
        self.beak_state = False
        self.get_logger().info('Torque control for S2/S3 switches initialized')

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
                        # Toggle on button press (momentary button: pressed = False)
                        # Detect falling edge (True -> False)
                        if self.previous_buttons[handler['button']] and not buttons[handler['button']]:
                            self.led_state = not self.led_state
                            brightness = 125 if self.led_state else 0
                            if self.can_bus is not None:
                                self._send_led_brightness(brightness)
                                self.get_logger().debug(f'LED brightness set to {brightness}')
                    elif handler.get('type') == 'beak':
                        if self.previous_buttons[handler['button']] and not buttons[handler['button']]:
                            self.beak_state = not self.beak_state
                            if self.can_bus is not None:
                                self._send_beak_command(self.beak_state)
                                self.get_logger().debug(f'Beak command sent: {"open" if self.beak_state else "close"}')
                    elif handler.get('type') == 'autonomy':
                        if self.previous_buttons[handler['button']] and not buttons[handler['button']]:
                            next_state = not self.autonomy_enabled
                            handler['service'].call_async(SetBool.Request(data=next_state))
                            self.get_logger().debug(f"Autonomy {'enabled' if next_state else 'disabled'} via button press")
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

        # Torque control for inter-module joints based on S2/S3 switch state
        # When switch is ON (down position, True), enable torque for the corresponding joint
        # When switch is OFF (up position, False), disable torque
        self.get_logger().debug(f'S2={s2}, prev_s2={self.prev_s2}, S3={s3}, prev_s3={self.prev_s3}, can_bus={self.can_bus is not None}')
        if s2 != self.prev_s2:
            # S2 controls module 2 (middle module) - joint motors are bits 2 and 3
            # Bit 2 = joint-left (yaw), Bit 3 = joint-right (pitch)
            torque_bitfield = 0xFFFF
            if not s2:
                # Enable torque for both joint motors on module 2
                torque_bitfield ^= 0b1100
            if self.can_bus is not None:
                self._send_torque_enable(torque_bitfield, module_id=0x22)  # MK2_MOD2
                self.get_logger().info(f'S2 switch changed: torque {"enabled" if s2 else "disabled"} for module 2 joints (bitfield=0x{torque_bitfield:04X})')
            else:
                self.get_logger().warn('CAN bus not available - cannot send torque command for S2')
            self.prev_s2 = s2

        if s3 != self.prev_s3:
            # S3 controls module 3 (tail module) - joint motors are bits 2 and 3
            torque_bitfield = 0xFFFF
            if not s3:
                # Enable torque for both joint motors on module 3
                torque_bitfield ^= 0b1100
            if self.can_bus is not None:
                self._send_torque_enable(torque_bitfield, module_id=0x23)  # MK2_MOD3
                self.get_logger().info(f'S3 switch changed: torque {"enabled" if s3 else "disabled"} for module 3 joints (bitfield=0x{torque_bitfield:04X})')
            else:
                self.get_logger().warn('CAN bus not available - cannot send torque command for S3')
            self.prev_s3 = s3

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

    def _send_torque_enable(self, torque_bitfield: int, module_id: int) -> None:
        """Send TORQUE_ENABLE_DISABLE (0x74) CAN message to a module.
        
        Args:
            torque_bitfield: uint16 bitmask (1=enable, 0=disable per motor)
            module_id: Target module CAN ID (0x22 for MOD2, 0x23 for MOD3)
        """
        try:
            # CAN ID format: [msg_id:8][mod_id:8][unused:16] with EFF flag
            # msg_id = 0x74 (TORQUE_ENABLE_DISABLE)
            # mod_id = module_id (0x22 or 0x23)
            arb_id = (0x74 << 16) | (module_id << 8) | 0x00
            
            # Pack torque_bitfield as little-endian uint16 (2 bytes)
            data = struct.pack('<H', torque_bitfield)
            
            msg = can.Message(
                arbitration_id=arb_id,
                data=data,
                is_extended_id=True,
            )
            self.can_bus.send(msg)
            self.get_logger().debug(f'Sent TORQUE_ENABLE_DISABLE to module 0x{module_id:02X}: bitfield=0x{torque_bitfield:04X}')
        except Exception as e:
            self.get_logger().error(f'Failed to send torque enable command: {type(e).__name__}: {e}')

    def _send_led_brightness(self, brightness: int) -> None:
        """Send LED_HP_BRIGHTNESS (0x75) CAN message to module 1.
        
        Args:
            brightness: 0-255 brightness value
        """
        try:
            # CAN ID format: [msg_id:8][mod_id:8][unused:16] with EFF flag
            # msg_id = 0x75 (LED_HP_BRIGHTNESS)
            # mod_id = 0x21 (MK2_MOD1)
            arb_id = (0x75 << 16) | (0x21 << 8) | 0x00
            
            # Pack brightness as uint8 (1 byte)
            data = struct.pack('<B', brightness)
            
            msg = can.Message(
                arbitration_id=arb_id,
                data=data,
                is_extended_id=True,
            )
            self.can_bus.send(msg)
            self.get_logger().debug(f'Sent LED_HP_BRIGHTNESS to module 0x21: brightness={brightness}')
        except Exception as e:
            self.get_logger().error(f'Failed to send LED brightness command: {type(e).__name__}: {e}')

    def _send_beak_command(self, open_beak: bool) -> None:
        """Send ARM_ROLL_6_SETPOINT (0x5B) CAN message to module 1 for beak control.
        
        Args:
            open_beak: True to open, False to close
        """
        try:
            # CAN ID format: [msg_id:8][mod_id:8][unused:16] with EFF flag
            # msg_id = 0x5B (ARM_ROLL_6_SETPOINT)
            # mod_id = 0x21 (MK2_MOD1)
            arb_id = (0x5B << 16) | (0x21 << 8) | 0x00
            
            # Pack as int32: 0=close, 1=open
            value = 1 if open_beak else 0
            data = struct.pack('<i', value)
            
            msg = can.Message(
                arbitration_id=arb_id,
                data=data,
                is_extended_id=True,
            )
            # Retry up to 3 times for reliability
            for attempt in range(3):
                try:
                    self.can_bus.send(msg)
                    self.get_logger().debug(f'Sent ARM_ROLL_6_SETPOINT to module 0x21: {"open" if open_beak else "close"}')
                    break
                except can.CanError as e:
                    if attempt == 2:
                        raise
                    self.get_logger().warn(f'Beak send attempt {attempt+1} failed, retrying: {e}')
                    time.sleep(0.01)
        except Exception as e:
            self.get_logger().error(f'Failed to send beak command: {type(e).__name__}: {e}')


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
