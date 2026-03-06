#!/usr/bin/env python3
import subprocess
import traceback

import rclpy
from geometry_msgs.msg import TwistStamped, Vector3
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveitController(Node):
    """ROS2 node for controlling the arm using MoveIt Servo.

    This node acts as an interface between high-level velocity commands and
    MoveIt Servo. It handles:
    - Converting Vector3 velocity commands to TwistStamped messages
    - Switching between linear and angular velocity control modes
    - Mirroring specific joint states to fix MoveIt issue with non-arm joints
    - Controlling the arm's beak (gripper) mechanism

    Subscribed Topics:
        /mk2_arm_vel (geometry_msgs/Vector3): Velocity commands for the arm
        /joint_states (sensor_msgs/JointState): Current joint states of the robot

    Published Topics:
        /moveit_servo_node/delta_twist_cmds (geometry_msgs/TwistStamped):
            Velocity commands for MoveIt Servo
        /arm_joint_states (sensor_msgs/JointState): Filtered joint states

    Services:
        /moveit_controller/switch_vel (std_srvs/SetBool):
            Switch between linear and angular velocity modes
        /moveit_controller/close_beak (std_srvs/SetBool):
            Open or close the arm's beak
        /moveit_controller/home (std_srvs/SetBool):
            Move the arm to its home position (all joints at 0)

    Parameters:
        planning_frame_id (str): Frame ID for motion planning (default: 'arm_base_link')
    """

    def __init__(self):
        super().__init__('moveit_controller')

        self.linear_vel_enabled = True
        self.create_service(SetBool, '/moveit_controller/switch_vel', self.switch_vel_type)
        self.create_service(SetBool, '/moveit_controller/close_beak', self.handle_beak)
        self.create_service(SetBool, '/moveit_controller/home', self.handle_home)

        self.planning_frame_id = (
            self.declare_parameter('planning_frame_id', 'arm_base_link')
            .get_parameter_value()
            .string_value
        )

        self.create_subscription(Vector3, '/mk2_arm_vel', self.handle_velocities, 10)
        self.speed_pub = self.create_publisher(
            TwistStamped, '/moveit_servo_node/delta_twist_cmds', 10
        )

        # Direct trajectory publisher for home command (bypasses Servo)
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/mk2_arm_controller/joint_trajectory', 10
        )

        self.current_joints = {}

        self.mirror_pub = self.create_publisher(JointState, '/arm_joint_states', 10)
        self.create_subscription(JointState, '/joint_states', self.mirror_states, 10)
        self.state_to_mirror = [
            'mod1__base_pitch_arm_joint',
            'mod1__base_roll_arm_joint',
            'mod1__elbow_pitch_arm_joint',
            'mod1__forearm_roll_arm_joint',
            'mod1__wrist_pitch_arm_joint',
            'mod1__wrist_roll_arm_joint',
        ]

        # Home position (all joints at 0) — matches SRDF "home" state.
        # Order: base_pitch, base_roll, elbow_pitch, forearm_roll, wrist_pitch, wrist_roll
        self.home_positions = [0.0] * len(self.state_to_mirror)

        # Diagnostic counters
        self._cmd_count = 0
        self._mirror_count = 0
        self._traj_out_count = 0
        self._last_diag_time = self.get_clock().now()
        self.create_timer(5.0, self._log_diagnostics)

        # Servo status monitoring
        self._servo_status = -1  # -1 = INVALID (not started)
        self._servo_status_names = {
            -1: 'INVALID',
            1: 'NO_WARNING',
            2: 'DECELERATE_FOR_SINGULARITY',
            3: 'DECELERATE_FOR_COLLISION',
            4: 'HALT_FOR_SINGULARITY',
            5: 'HALT_FOR_COLLISION',
            6: 'JOINT_BOUND',
        }
        self.create_subscription(Int8, '/moveit_servo_node/status', self._on_servo_status, 10)

        # Monitor trajectories actually output by Servo
        self.create_subscription(
            JointTrajectory,
            '/mk2_arm_controller/joint_trajectory',
            self._on_trajectory_out,
            10,
        )

        # Track velocity command gaps
        self._last_vel_cmd_time = None
        self._vel_gap_logged = False

        # Start MoveIt Servo after a short delay (needs time to initialize)
        self._servo_started = False
        self._start_servo_client = self.create_client(Trigger, '/moveit_servo_node/start_servo')
        self.create_timer(3.0, self._start_servo_once)

        self.get_logger().info('Node Moveit Controller started successfully')

    def mirror_states(self, msg: JointState):
        """Mirror specific joint states to a filtered topic.

        Filters the incoming joint states message to only include joints
        specified in self.state_to_mirror and publishes them to a separate topic.
        This fixes issues with MoveIt when non-arm joints are present.

        Args:
            msg (JointState): The incoming joint states message containing all joints.

        Note:
            Updates self.current_joints dictionary with all received joint positions.
        """
        fmsg = JointState()
        fmsg.header = msg.header
        is_to_pub = False
        for i, name in enumerate(msg.name):
            self.current_joints[name] = msg.position[i]
            if name in self.state_to_mirror:
                is_to_pub = True
                fmsg.name.append(name)
                fmsg.position.append(msg.position[i])
                if msg.velocity != []:
                    fmsg.velocity.append(msg.velocity[i])
                if msg.effort != []:
                    fmsg.effort.append(0.0)
        if is_to_pub:
            self.mirror_pub.publish(fmsg)
            self._mirror_count += 1

    @staticmethod
    def _apply_deadzone(value: float, threshold: float = 0.08) -> float:
        """Zero out values below the deadzone threshold to filter joystick noise."""
        return value if abs(value) > threshold else 0.0

    def handle_velocities(self, msg: Vector3):
        """Convert incoming velocity commands to MoveIt Servo format.

        Remaps joystick axes to arm_base_link frame using "crane" control scheme:
          Linear mode:
            - Joy Y (fwd/back) → arm Z (up/down)    — push fwd to lift
            - Joy X (left/right) → arm Y (left/right, inverted)
            - Joy Z (twist)      → arm X (extend/retract)
          Angular mode:
            - Joy Y (fwd/back) → angular Y (pitch)
            - Joy X (left/right) → angular Z (yaw, inverted)
            - Joy Z (twist)      → angular X (roll)

        Args:
            msg (Vector3): Raw joystick input (x=left/right, y=forward/back, z=twist).
        """
        # Apply deadzone to filter joystick noise
        joy_x = self._apply_deadzone(msg.x)  # left(−)/right(+)
        joy_y = self._apply_deadzone(msg.y)  # forward(+)/back(−)
        joy_z = self._apply_deadzone(msg.z)  # twist CW(+)/CCW(−)

        # Skip publishing if all axes are zero (joystick at rest)
        if joy_x == 0.0 and joy_y == 0.0 and joy_z == 0.0:
            self._vel_gap_logged = False
            return

        # Log when velocity commands resume after a gap
        now = self.get_clock().now()
        if not self._vel_gap_logged:
            gap_ms = 0.0
            if self._last_vel_cmd_time is not None:
                gap_ms = (now - self._last_vel_cmd_time).nanoseconds / 1e6
            self.get_logger().info(
                f'[VEL] Joystick active: x={joy_x:.2f} y={joy_y:.2f} z={joy_z:.2f} '
                f'gap={gap_ms:.0f}ms servo_status='
                f'{self._servo_status_names.get(self._servo_status, "?")}'
            )
            self._vel_gap_logged = True
        self._last_vel_cmd_time = now

        servo_msg = TwistStamped()
        servo_msg.header.stamp = self.get_clock().now().to_msg()
        servo_msg.header.frame_id = self.planning_frame_id

        if self.linear_vel_enabled:
            # "Crane" control — arm_base_link frame (X=fwd, Y=left, Z=up)
            servo_msg.twist.linear.x = joy_z  # twist → extend/retract
            servo_msg.twist.linear.y = -joy_x  # left/right (inverted)
            servo_msg.twist.linear.z = joy_y  # forward → up, backward → down
        else:
            # Angular velocity in arm_base_link frame
            servo_msg.twist.angular.x = joy_z  # twist → roll
            servo_msg.twist.angular.y = joy_y  # forward → pitch up
            servo_msg.twist.angular.z = -joy_x  # right → yaw CW

        self.speed_pub.publish(servo_msg)
        self._cmd_count += 1

    def switch_vel_type(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Service callback to switch between linear and angular velocity modes.

        Args:
            request (SetBool.Request): Service request containing:
                - data (bool): True for linear velocity, False for angular velocity
            response (SetBool.Response): Service response to be filled

        Returns:
            SetBool.Response: Response indicating success and current velocity mode
        """
        self.linear_vel_enabled = request.data

        response.success = True
        response.message = (
            'Input velocity set to LINEAR'
            if self.linear_vel_enabled
            else 'Input velocity set to ANGULAR'
        )
        self.get_logger().info(response.message)
        return response

    def handle_beak(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Service callback to control the arm's beak (gripper) mechanism.

        Args:
            request (SetBool.Request): Service request containing:
                - data (bool): True to close the beak, False to open it
            response (SetBool.Response): Service response to be filled

        Returns:
            SetBool.Response: Response indicating success and the requested action

        Note:
            Uses cansend to directly send CAN commands because the hardware
            interface does not support the beak's int32 setpoint type.
            CAN ID 005B2100: msg_id=0x5B, mod_id=0x21 (module 1).
            Payload: int32 little-endian, 0=CLOSE, 1=OPEN.
        """
        if request.data:
            # CLOSE: int32(0) in little-endian
            can_data = '00000000'
        else:
            # OPEN: int32(1) in little-endian
            can_data = '01000000'

        can_frame = f'005B2100#{can_data}'
        try:
            subprocess.run(
                ['cansend', 'can0', can_frame],
                check=True,
                timeout=1.0,
            )
            response.success = True
        except FileNotFoundError:
            response.success = False
            response.message = 'cansend not found in PATH'
            self.get_logger().error(response.message)
            return response
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError) as e:
            response.success = False
            response.message = f'cansend failed: {e}'
            self.get_logger().error(response.message)
            return response

        action = 'CLOSE' if request.data else 'OPEN'
        response.message = f'Sent CAN beak {action} ({can_frame})'
        self.get_logger().info(response.message)
        return response

    def handle_home(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Service callback to move the arm to its home position.

        Sends a JointTrajectory message directly to the arm controller
        (bypassing MoveIt Servo) to move all joints to their home positions.

        Args:
            request (SetBool.Request): Service request (data field is ignored,
                home always triggers on every call)
            response (SetBool.Response): Service response to be filled

        Returns:
            SetBool.Response: Response indicating success
        """
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = list(self.state_to_mirror)

        point = JointTrajectoryPoint()
        point.positions = list(self.home_positions)
        point.velocities = [0.0] * len(self.state_to_mirror)
        point.time_from_start.sec = 3  # 3 seconds to reach home
        point.time_from_start.nanosec = 0
        traj_msg.points = [point]

        self.trajectory_pub.publish(traj_msg)
        response.success = True
        response.message = 'Arm moving to home position'
        self.get_logger().info(response.message)
        return response

    def _log_diagnostics(self):
        """Periodic diagnostic logging for monitoring command throughput."""
        now = self.get_clock().now()
        dt = (now - self._last_diag_time).nanoseconds / 1e9
        if dt > 0:
            cmd_rate = self._cmd_count / dt
            mirror_rate = self._mirror_count / dt
            traj_rate = self._traj_out_count / dt
            joint_info = ', '.join(
                f'{k}: {v:.3f}'
                for k, v in self.current_joints.items()
                if k in self.state_to_mirror
            )
            status_name = self._servo_status_names.get(
                self._servo_status, f'UNKNOWN({self._servo_status})'
            )
            self.get_logger().info(
                f'[DIAG] cmd_rate={cmd_rate:.1f}Hz, '
                f'mirror_rate={mirror_rate:.1f}Hz, '
                f'traj_out_rate={traj_rate:.1f}Hz, '
                f'servo_status={status_name}, '
                f'mode={"LINEAR" if self.linear_vel_enabled else "ANGULAR"}, '
                f'joints=[{joint_info}]'
            )
        self._cmd_count = 0
        self._mirror_count = 0
        self._traj_out_count = 0
        self._last_diag_time = now

    def _on_servo_status(self, msg: Int8):
        """Track MoveIt Servo status and log changes."""
        if msg.data != self._servo_status:
            old_name = self._servo_status_names.get(
                self._servo_status, f'UNKNOWN({self._servo_status})'
            )
            new_name = self._servo_status_names.get(msg.data, f'UNKNOWN({msg.data})')
            self.get_logger().warn(f'[SERVO] Status changed: {old_name} -> {new_name}')
            self._servo_status = msg.data

    def _on_trajectory_out(self, msg: JointTrajectory):
        """Monitor trajectories output by Servo or home command."""
        self._traj_out_count += 1

    def _start_servo_once(self):
        """Start MoveIt Servo by calling the /start_servo service (one-shot)."""
        if self._servo_started:
            return
        if not self._start_servo_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('Waiting for /moveit_servo_node/start_servo service...')
            return
        future = self._start_servo_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_servo_started)

    def _on_servo_started(self, future):
        """Callback when start_servo service responds."""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('MoveIt Servo started successfully')
                self._servo_started = True
            else:
                self.get_logger().warn(f'MoveIt Servo start failed: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Error starting MoveIt Servo: {e}')


def main(args=None):
    rclpy.init(args=args)
    try:
        moveit_controller = MoveitController()
        rclpy.spin(moveit_controller)
    except KeyboardInterrupt:
        rclpy.logging.get_logger('moveit_controller').warn(
            'Moveit controller node interrupted by user'
        )
    except Exception as err:
        rclpy.logging.get_logger('moveit_controller').fatal(
            f'Error in the Moveit controller node: {str(err)}\n{traceback.format_exc()}'
        )
    else:
        moveit_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
