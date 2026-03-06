#!/usr/bin/env python3
"""MoveIt Servo pipeline test for the ReseQ MK2 arm.

Tests the full teleoperation pipeline that the remote controller joystick uses:
  TwistStamped → MoveIt Servo → IK → trajectory controller → hardware interface → CAN

This is the Servo counterpart to arm_test_scenario.py (which uses direct trajectory
commands). Both should produce similar physical arm movement; if this test is wobbly
but arm_test_scenario is smooth, the problem is in the Servo/IK pipeline.

Must be run while the full reseq launch is active.

Usage (inside the container, after sourcing):
    python3 servo_test_scenario.py
"""

import subprocess
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = [
    'mod1__base_pitch_arm_joint',
    'mod1__base_roll_arm_joint',
    'mod1__elbow_pitch_arm_joint',
    'mod1__forearm_roll_arm_joint',
    'mod1__wrist_pitch_arm_joint',
    'mod1__wrist_roll_arm_joint',
]

BEAK_CAN_ID = '005B2100'


class ServoTestScenario(Node):
    """Exercises the MoveIt Servo pipeline with scripted velocity commands."""

    def __init__(self):
        super().__init__('servo_test_scenario')

        # Servo velocity input — same topic as moveit_controller uses
        self.twist_pub = self.create_publisher(
            TwistStamped, '/moveit_servo_node/delta_twist_cmds', 10
        )

        # Direct trajectory for home (bypasses Servo, same as handle_home)
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/mk2_arm_controller/joint_trajectory', 10
        )

        # Service clients for button features
        self.start_servo_client = self.create_client(Trigger, '/moveit_servo_node/start_servo')
        self.switch_vel_client = self.create_client(SetBool, '/moveit_controller/switch_vel')
        self.beak_client = self.create_client(SetBool, '/moveit_controller/close_beak')
        self.home_client = self.create_client(SetBool, '/moveit_controller/home')

        self.planning_frame = 'arm_base_link'
        time.sleep(1.0)
        self.get_logger().info('=== MOVEIT SERVO PIPELINE TEST ===')

    # ── helpers ────────────────────────────────────────────────────────

    def ensure_servo_started(self):
        """Call /start_servo service (idempotent — safe if already running)."""
        if not self.start_servo_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                'start_servo service not available — Servo may already be running'
            )
            return
        future = self.start_servo_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            self.get_logger().info(f'Servo start: {future.result().message}')
        else:
            self.get_logger().warn('Servo start timed out')

    def send_twist(
        self,
        linear: tuple[float, float, float] = (0.0, 0.0, 0.0),
        angular: tuple[float, float, float] = (0.0, 0.0, 0.0),
        duration_sec: float = 2.0,
        rate_hz: float = 12.5,
        label: str = '',
    ):
        """Publish continuous TwistStamped for *duration_sec* at *rate_hz*.

        Uses the same frame (arm_base_link) and topic as moveit_controller.

        Velocity values are in the *unitless* [-1, 1] range — MoveIt Servo
        multiplies by ``scale.linear`` / ``scale.rotational`` from servo_config.
        """
        self.get_logger().info(
            f'[TWIST] {label}  lin={linear} ang={angular} dur={duration_sec:.1f}s'
        )

        period = 1.0 / rate_hz
        elapsed = 0.0
        while elapsed < duration_sec:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.planning_frame
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = linear
            msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = angular
            self.twist_pub.publish(msg)
            time.sleep(period)
            elapsed += period

        time.sleep(0.8)  # let arm settle after velocity stream stops

    def send_home(self, duration_sec: float = 3.0, label: str = 'Home'):
        """Return all joints to 0 via direct JointTrajectory (same as handle_home)."""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = [0.0] * len(JOINT_NAMES)
        pt.velocities = [0.0] * len(JOINT_NAMES)
        pt.time_from_start.sec = int(duration_sec)
        msg.points = [pt]

        self.get_logger().info(f'[HOME] {label} (dur={duration_sec:.1f}s)')
        self.traj_pub.publish(msg)
        time.sleep(duration_sec + 0.5)

    def set_beak_can(self, close: bool, label: str):
        """Control beak via direct cansend (same as handle_beak)."""
        data_hex = '00000000' if close else '01000000'
        frame = f'{BEAK_CAN_ID}#{data_hex}'
        self.get_logger().info(
            f'[BEAK-CAN] {label} ({"CLOSE" if close else "OPEN"}) — cansend can0 {frame}'
        )
        result = subprocess.run(['cansend', 'can0', frame], capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().error(f'cansend failed: {result.stderr}')
        time.sleep(1.5)

    def set_beak_service(self, close: bool, label: str):
        """Control beak via /moveit_controller/close_beak service."""
        if not self.beak_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'close_beak service not available for: {label}')
            return
        req = SetBool.Request(data=close)
        future = self.beak_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() is not None:
            self.get_logger().info(
                f'[BEAK-SRV] {label}: success={future.result().success} '
                f'msg={future.result().message}'
            )
        else:
            self.get_logger().error(f'[BEAK-SRV] {label}: timed out')
        time.sleep(1.5)

    def call_switch_vel(self, linear: bool, label: str):
        """Switch arm velocity mode via /moveit_controller/switch_vel service."""
        if not self.switch_vel_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'switch_vel service not available for: {label}')
            return
        req = SetBool.Request(data=linear)
        future = self.switch_vel_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() is not None:
            self.get_logger().info(
                f'[SWITCH] {label}: success={future.result().success} '
                f'msg={future.result().message}'
            )
        else:
            self.get_logger().error(f'[SWITCH] {label}: timed out')

    # ── scenario ───────────────────────────────────────────────────────

    def run_scenario(self):
        """Execute the full Servo pipeline test."""

        # ── Phase 0: Start Servo ──────────────────────────────────────
        self.ensure_servo_started()
        time.sleep(2.0)
        self.get_logger().info('Starting servo pipeline test in 3 seconds...')
        time.sleep(3.0)

        # ================================================================
        # PHASE 1: LINEAR VELOCITY (Cartesian translation in arm_base_link)
        # ================================================================
        self.get_logger().info('=== PHASE 1: LINEAR VELOCITY ===')

        self.send_twist(
            linear=(0.0, 0.0, 0.6),
            duration_sec=2.0,
            label='L1. Move UP (+Z)',
        )

        self.send_twist(
            linear=(0.0, 0.0, -0.6),
            duration_sec=2.0,
            label='L2. Move DOWN (-Z)',
        )

        self.send_twist(
            linear=(0.4, 0.0, 0.0),
            duration_sec=2.0,
            label='L3. Extend FORWARD (+X)',
        )

        self.send_twist(
            linear=(-0.4, 0.0, 0.0),
            duration_sec=2.0,
            label='L4. Retract BACKWARD (-X)',
        )

        self.send_twist(
            linear=(0.0, 0.4, 0.0),
            duration_sec=2.0,
            label='L5. Move LEFT (+Y)',
        )

        self.send_twist(
            linear=(0.0, -0.4, 0.0),
            duration_sec=2.0,
            label='L6. Move RIGHT (-Y)',
        )

        self.send_home(label='Home after linear tests')

        # ================================================================
        # PHASE 2: ANGULAR VELOCITY (Cartesian rotation in arm_base_link)
        # ================================================================
        self.get_logger().info('=== PHASE 2: ANGULAR VELOCITY ===')

        self.send_twist(
            angular=(0.0, 0.5, 0.0),
            duration_sec=2.0,
            label='R1. Pitch UP (+Y ang)',
        )

        self.send_twist(
            angular=(0.0, -0.5, 0.0),
            duration_sec=2.0,
            label='R2. Pitch DOWN (-Y ang)',
        )

        self.send_twist(
            angular=(0.0, 0.0, 0.5),
            duration_sec=2.0,
            label='R3. Yaw LEFT (+Z ang)',
        )

        self.send_twist(
            angular=(0.0, 0.0, -0.5),
            duration_sec=2.0,
            label='R4. Yaw RIGHT (-Z ang)',
        )

        self.send_twist(
            angular=(0.5, 0.0, 0.0),
            duration_sec=2.0,
            label='R5. Roll CW (+X ang)',
        )

        self.send_twist(
            angular=(-0.5, 0.0, 0.0),
            duration_sec=2.0,
            label='R6. Roll CCW (-X ang)',
        )

        self.send_home(label='Home after angular tests')

        # ================================================================
        # PHASE 3: BEAK TEST (both direct CAN and service)
        # ================================================================
        self.get_logger().info('=== PHASE 3: BEAK ===')

        # Test via direct cansend first (known to work in arm_test_scenario)
        self.set_beak_can(close=False, label='3a. Open  (cansend)')
        self.set_beak_can(close=True, label='3b. Close (cansend)')

        # Test via service (same path as remote controller button)
        self.set_beak_service(close=False, label='3c. Open  (service)')
        self.set_beak_service(close=True, label='3d. Close (service)')

        # ================================================================
        # PHASE 4: VELOCITY MODE SWITCH (red button)
        # ================================================================
        self.get_logger().info('=== PHASE 4: MODE SWITCH ===')

        # Currently LINEAR (default). Switch to ANGULAR.
        self.call_switch_vel(False, '4a. Switch to ANGULAR')
        time.sleep(1.0)

        self.get_logger().info(
            'Verify: same twist now maps to ANGULAR motion (pitch instead of translation)'
        )
        self.send_twist(
            angular=(0.0, 0.4, 0.0),
            duration_sec=1.5,
            label='4b. Pitch in ANGULAR mode',
        )

        # Switch back to LINEAR
        self.call_switch_vel(True, '4c. Switch back to LINEAR')
        time.sleep(1.0)

        self.send_twist(
            linear=(0.0, 0.0, 0.4),
            duration_sec=1.5,
            label='4d. Move up in LINEAR mode',
        )

        self.send_home(label='Final home')

        # ================================================================
        # PHASE 5: COMBINED SERVO PICK-AND-PLACE
        # ================================================================
        self.get_logger().info('=== PHASE 5: SERVO PICK-AND-PLACE ===')

        # Unfold: move end-effector down and forward
        self.send_twist(
            linear=(0.2, 0.0, -0.3),
            duration_sec=3.0,
            label='5a. Reach down & forward',
        )

        # Open beak
        self.set_beak_can(close=False, label='5b. Open beak')

        # Approach
        self.send_twist(
            linear=(0.15, 0.0, -0.15),
            duration_sec=1.5,
            label='5c. Fine approach',
        )

        # Grab
        self.set_beak_can(close=True, label='5d. GRAB')

        # Lift
        self.send_twist(
            linear=(0.0, 0.0, 0.5),
            duration_sec=2.0,
            label='5e. Lift up',
        )

        # Rotate
        self.send_twist(
            angular=(0.0, 0.0, 0.4),
            duration_sec=2.0,
            label='5f. Rotate left',
        )

        # Lower
        self.send_twist(
            linear=(0.0, 0.0, -0.3),
            duration_sec=2.0,
            label='5g. Lower to placement',
        )

        # Release
        self.set_beak_can(close=False, label='5h. RELEASE')

        # Retract and home
        self.send_home(duration_sec=3.5, label='5i. Return home')

        self.get_logger().info('=== SERVO PIPELINE TEST COMPLETE ===')


def main(args=None):
    rclpy.init(args=args)
    node = ServoTestScenario()
    try:
        node.run_scenario()
    except KeyboardInterrupt:
        node.get_logger().warn('Scenario interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
