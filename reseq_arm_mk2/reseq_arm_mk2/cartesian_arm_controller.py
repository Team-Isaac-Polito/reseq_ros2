#!/usr/bin/env python3
"""
Jacobian-based Cartesian arm controller for reseq MK2.

Replaces MoveIt Servo with a direct Jacobian velocity loop that:
  - Reads /joint_states at ~33 Hz
  - Converts joystick Cartesian velocity → joint velocities via J^†
  - Applies per-joint limit clipping (offending joint is zeroed, others continue)
  - Publishes to /mk2_arm_controller/joint_trajectory
  - Exposes services for home, beak, and mode switching

Issue #106 / #107 implementation.
"""

import traceback
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ── Optional KDL ──────────────────────────────────────────────────────────────
_HAS_KDL = False
_KDL_ERR = ''
try:
    import PyKDL as kdl
except ImportError as _e:
    _KDL_ERR = f'PyKDL import failed: {_e}'
else:
    try:
        from kdl_parser_py.urdf import treeFromString
    except ImportError as _e:
        _KDL_ERR = f'kdl_parser_py import failed: {_e}'
    else:
        _HAS_KDL = True


class CartesianArmController(Node):
    """
    Jacobian-based Cartesian arm controller.

    Subscribed Topics
    -----------------
    /mk2_arm_vel          geometry_msgs/Vector3   joystick Cartesian input  [-1, 1]
    /joint_states         sensor_msgs/JointState

    Published Topics
    ----------------
    /mk2_arm_controller/joint_trajectory  trajectory_msgs/JointTrajectory
    reseq/module33/mk2_arm/beak/setpoint  std_msgs/Int32

    Services
    --------
    /cartesian_arm_controller/go_home       std_srvs/Trigger
    /cartesian_arm_controller/switch_vel    std_srvs/SetBool  True=linear, False=angular
    /cartesian_arm_controller/set_mode      std_srvs/SetBool  True=velocity, False=pos-increment
    /cartesian_arm_controller/close_beak    std_srvs/SetBool  True=close, False=open

    Parameters
    ----------
    robot_description   str    URDF XML (forwarded from robot_state_publisher in launch file)
    chain_root          str    'arm_base_link'
    chain_tip           str    'arm_roll_wrist_link'
    command_frame       str    'arm_base_link'
    trajectory_topic    str    '/mk2_arm_controller/joint_trajectory'
    control_rate        float  33.0   Hz
    max_cartesian_vel   float  0.3    m/s  (input is scaled by this)
    max_joint_vel       float  1.0    rad/s per joint
    home_duration_sec   float  3.0    s
    trajectory_horizon_sec float 1.00  s
    command_mode        str    'trajectory'  or 'velocity'
    deadzone            float  0.02   ignore commands below this magnitude
    jacobian_damping    float 0.08   damped least-squares regularization
    """

    # ── Constants ──────────────────────────────────────────────────────────────

    JOINT_NAMES = [
        'mod1__base_pitch_arm_joint',
        'mod1__base_roll_arm_joint',
        'mod1__elbow_pitch_arm_joint',
        'mod1__forearm_roll_arm_joint',
        'mod1__wrist_pitch_arm_joint',
        'mod1__wrist_roll_arm_joint',
    ]
    N_JOINTS = len(JOINT_NAMES)
    HOME_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Fallback limits (±120°) – overridden by URDF when KDL loads successfully
    _LOWER_DEFAULT = np.full(6, -2.094)
    _UPPER_DEFAULT = np.full(6, 2.094)

    # Approximate link lengths for the numerical-FK fallback (metres).
    # Measure from the real robot or the URDF if KDL is unavailable.
    _L1 = 0.10  # base vertical offset
    _L2 = 0.22  # upper-arm segment (pitch_forearm)
    _L3 = 0.20  # forearm segment  (roll_forearm)
    _L4 = 0.10  # wrist to end-effector

    def __init__(self):
        super().__init__('cartesian_arm_controller')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('robot_description', '')
        self.declare_parameter('chain_root', 'arm_base_link')
        self.declare_parameter('chain_tip', 'arm_roll_wrist_link')
        self.declare_parameter('command_frame', 'arm_base_link')
        self.declare_parameter('trajectory_topic', '/mk2_arm_controller/joint_trajectory')
        self.declare_parameter('control_rate', 33.0)
        self.declare_parameter('max_cartesian_vel', 0.3)
        self.declare_parameter('max_joint_vel', 1.0)
        self.declare_parameter('home_duration_sec', 3.0)
        self.declare_parameter('trajectory_horizon_sec', 0.1)  # shorter horizon for responsiveness
        self.declare_parameter('command_mode', 'trajectory')
        self.declare_parameter('deadzone', 0.02)
        self.declare_parameter('jacobian_damping', 0.08)

        # ── Runtime state ─────────────────────────────────────────────────────
        self._q: np.ndarray | None = None  # current joint positions (rad)
        self._q_cmd: np.ndarray | None = None  # integrator state for trajectory mode
        self._cmd_vel = np.zeros(3)  # latest Cartesian command (normalised)
        self._linear_mode = True  # True=linear cart, False=angular
        self._velocity_mode = True  # True=velocity-while-held, False=pos-increment
        self._moving = False

        # ── Joint limits ──────────────────────────────────────────────────────
        self._q_lo = self._LOWER_DEFAULT.copy()
        self._q_hi = self._UPPER_DEFAULT.copy()

        # ── KDL ───────────────────────────────────────────────────────────────
        self._kdl_chain: 'kdl.Chain | None' = None
        self._fk_solver: 'kdl.ChainFkSolverPos_recursive | None' = None
        self._jac_solver: 'kdl.ChainJntToJacSolver | None' = None
        self._command_frame = (
            self.get_parameter('command_frame').get_parameter_value().string_value
        )
        self._load_kdl()

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self.create_subscription(Vector3, '/mk2_arm_vel', self._cb_vel, 10)
        self.create_subscription(JointState, '/joint_states', self._cb_joint_state, 10)

        traj_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value
        self._command_mode = self.get_parameter('command_mode').get_parameter_value().string_value
        self._traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)
        self._vel_pub = self.create_publisher(
            Float64MultiArray, '/joint_group_velocity_controller/commands', 10
        )
        self._beak_pub = self.create_publisher(Int32, 'reseq/module33/mk2_arm/beak/setpoint', 10)

        self.create_service(Trigger, '/cartesian_arm_controller/go_home', self._srv_home)
        self.create_service(SetBool, '/cartesian_arm_controller/switch_vel', self._srv_switch_vel)
        self.create_service(SetBool, '/cartesian_arm_controller/set_mode', self._srv_set_mode)
        self.create_service(SetBool, '/cartesian_arm_controller/close_beak', self._srv_beak)

        rate = self.get_parameter('control_rate').get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            'CartesianArmController started '
            f'(KDL: {"YES ✓" if self._kdl_chain else "NO – numerical fallback"}, '
            f'command_frame={self._command_frame})'
        )

    # ── KDL setup ─────────────────────────────────────────────────────────────

    def _load_kdl(self):
        if not _HAS_KDL:
            self.get_logger().warn(f'KDL unavailable ({_KDL_ERR}). Using numerical Jacobian.')
            return

        urdf = self.get_parameter('robot_description').get_parameter_value().string_value
        if not urdf:
            self.get_logger().warn(
                'robot_description is empty → KDL disabled. '
                'Make sure the launch file forwards robot_description to this node.'
            )
            return

        ok, tree = treeFromString(urdf)
        if not ok:
            self.get_logger().error('treeFromString failed – check URDF validity.')
            return

        root = self.get_parameter('chain_root').get_parameter_value().string_value
        tip = self.get_parameter('chain_tip').get_parameter_value().string_value
        chain = kdl.Chain()
        if not tree.getChain(root, tip, chain):
            self.get_logger().error(f'KDL getChain({root} → {tip}) failed.')
            return

        self._kdl_chain = chain
        self._fk_solver = kdl.ChainFkSolverPos_recursive(chain)
        self._jac_solver = kdl.ChainJntToJacSolver(chain)
        self._parse_urdf_limits(urdf)

        self.get_logger().info(
            f'KDL chain {root} → {tip}: {chain.getNrOfJoints()} movable joint(s).'
        )

    def _parse_urdf_limits(self, urdf: str):
        """Extract joint limits from the URDF XML for our 6 joints."""
        try:
            root = ET.fromstring(urdf)
            for jel in root.findall('joint'):
                name = jel.get('name', '')
                if name not in self.JOINT_NAMES:
                    continue
                idx = self.JOINT_NAMES.index(name)
                lim = jel.find('limit')
                if lim is None:
                    continue
                self._q_lo[idx] = float(lim.get('lower', self._q_lo[idx]))
                self._q_hi[idx] = float(lim.get('upper', self._q_hi[idx]))
            self.get_logger().info(
                f'Joint limits loaded → lower={np.round(self._q_lo, 3)}, '
                f'upper={np.round(self._q_hi, 3)}'
            )
        except Exception as e:
            self.get_logger().warn(f'Could not parse joint limits from URDF: {e}')

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _cb_joint_state(self, msg: JointState):
        pos_map = dict(zip(msg.name, msg.position))
        try:
            self._q = np.array([pos_map[n] for n in self.JOINT_NAMES])
        except KeyError:
            pass  # Not all joints present yet – wait silently

    def _cb_vel(self, msg: Vector3):
        self._cmd_vel = np.array([msg.x, msg.y, msg.z])

    # ── Jacobian ──────────────────────────────────────────────────────────────

    def _jacobian_kdl(self, q: np.ndarray) -> np.ndarray | None:
        """Return the 3×N linear-velocity portion of the KDL Jacobian."""
        n = self._kdl_chain.getNrOfJoints()
        q_kdl = kdl.JntArray(n)
        for i in range(min(n, self.N_JOINTS)):
            q_kdl[i] = float(q[i])

        jac = kdl.Jacobian(n)
        if self._jac_solver.JntToJac(q_kdl, jac) < 0:
            self.get_logger().warn('KDL JntToJac returned error.')
            return None

        J = np.zeros((3, n))
        for c in range(n):
            twist = jac.getColumn(c)
            J[0, c] = twist.vel.x()
            J[1, c] = twist.vel.y()
            J[2, c] = twist.vel.z()

        return J

    def _fk_position(self, q: np.ndarray) -> np.ndarray:
        """Return tip position from the real KDL chain."""
        if self._kdl_chain is not None and self._fk_solver is not None:
            n = self._kdl_chain.getNrOfJoints()
            q_kdl = kdl.JntArray(n)
            for i in range(min(n, self.N_JOINTS)):
                q_kdl[i] = float(q[i])

            tip_frame = kdl.Frame()
            if self._fk_solver.JntToCart(q_kdl, tip_frame) >= 0:
                return np.array([tip_frame.p.x(), tip_frame.p.y(), tip_frame.p.z()])

        # Conservative fallback if KDL is unavailable or fails.
        return self._numerical_fk(q)

    def _numerical_fk(self, q: np.ndarray) -> np.ndarray:
        """Lightweight geometric fallback if KDL forward kinematics fails."""
        q0, q1, q2, _, q4, _ = q
        az = q1
        el0 = q0
        el2 = q0 + q2
        el4 = q0 + q2 + q4

        reach = self._L2 * np.cos(el0) + self._L3 * np.cos(el2) + self._L4 * np.cos(el4)
        x = np.cos(az) * reach
        y = np.sin(az) * reach
        z = self._L1 + self._L2 * np.sin(el0) + self._L3 * np.sin(el2) + self._L4 * np.sin(el4)
        return np.array([x, y, z])

    def _tip_rotation(self, q: np.ndarray) -> np.ndarray:
        """Return the tip rotation matrix in the base frame."""
        if self._kdl_chain is not None and self._fk_solver is not None:
            n = self._kdl_chain.getNrOfJoints()
            q_kdl = kdl.JntArray(n)
            for i in range(min(n, self.N_JOINTS)):
                q_kdl[i] = float(q[i])

            tip_frame = kdl.Frame()
            if self._fk_solver.JntToCart(q_kdl, tip_frame) >= 0:
                return np.array(
                    [
                        [tip_frame.M[0, 0], tip_frame.M[0, 1], tip_frame.M[0, 2]],
                        [tip_frame.M[1, 0], tip_frame.M[1, 1], tip_frame.M[1, 2]],
                        [tip_frame.M[2, 0], tip_frame.M[2, 1], tip_frame.M[2, 2]],
                    ]
                )

        return np.eye(3)

    def _command_velocity_in_base(self, q: np.ndarray, cart_vel: np.ndarray) -> np.ndarray:
        """Cartesian commands are interpreted in the fixed base frame."""
        return cart_vel

    def _jacobian_numerical(self, q: np.ndarray, eps: float = 1e-4) -> np.ndarray:
        """Finite-difference Jacobian (3×N) using _fk_position."""
        p0 = self._fk_position(q)
        J = np.zeros((3, self.N_JOINTS))
        for i in range(self.N_JOINTS):
            dq = q.copy()
            dq[i] += eps
            J[:, i] = (self._fk_position(dq) - p0) / eps
        return J

    def _get_jacobian(self, q: np.ndarray) -> np.ndarray | None:
        if self._jac_solver is not None:
            return self._jacobian_kdl(q)
        return self._jacobian_numerical(q)

    # ── Main control loop ─────────────────────────────────────────────────────

    def _control_loop(self):
        """33 Hz: Cartesian velocity → joint trajectory step."""
        if self._q is None:
            return

        # Init the integrator state if missing
        if getattr(self, '_q_cmd', None) is None:
            self._q_cmd = self._q.copy()
            self._moving = False

        # ── Dead-zone ────────────────────────────────────────────────────────
        deadzone = self.get_parameter('deadzone').get_parameter_value().double_value
        if np.linalg.norm(self._cmd_vel) < deadzone:
            if getattr(self, '_moving', False):
                # Publish a stop command around current position
                self._q_cmd = self._q.copy()
                self._publish_traj(self._q_cmd.tolist(), [0.0] * self.N_JOINTS, 0.1)
                self._moving = False
            return

        # If we just started moving sync command to current measured position to avoid sudden jumps
        if not getattr(self, '_moving', False):
            self._q_cmd = self._q.copy()
            self._moving = True

        if not self._linear_mode:
            # Angular (orientation) mode – to be implemented in a future iteration
            # For now, silently skip to avoid unexpected motion.
            return

        max_cv = self.get_parameter('max_cartesian_vel').get_parameter_value().double_value
        max_jv = self.get_parameter('max_joint_vel').get_parameter_value().double_value
        rate = self.get_parameter('control_rate').get_parameter_value().double_value
        dt = 1.0 / rate

        # Scale joystick [-1,1] → [−max_cv, max_cv] m/s
        cart_vel_tool = self._cmd_vel * max_cv
        cart_vel = self._command_velocity_in_base(self._q, cart_vel_tool)

        # ── Jacobian pseudo-inverse ───────────────────────────────────────────
        J = self._get_jacobian(self._q)
        if J is None:
            return

        try:
            # Damped least-squares solution: dq = J^T (J J^T + λ² I)^-1 v
            damping = self.get_parameter('jacobian_damping').get_parameter_value().double_value
            JJt = J @ J.T
            dq = J.T @ np.linalg.solve(JJt + (damping * damping) * np.eye(J.shape[0]), cart_vel)
        except np.linalg.LinAlgError:
            self.get_logger().warn('Jacobian pseudo-inverse failed.')
            return

        # ── Uniform velocity scaling to preserve end-effector direction ──────
        peak = float(np.max(np.abs(dq)))
        scale = 1.0
        if peak > max_jv:
            scale = max_jv / peak
            dq = dq * scale

        # DEBUG PRINTS
        if not hasattr(self, '_log_ctr'):
            self._log_ctr = 0
        self._log_ctr += 1
        if self._log_ctr % 33 == 0 and self._moving:
            has_kdl = self._jac_solver is not None
            tip_pos = self._fk_position(self._q).round(3)
            tip_cmd = self._fk_position(self._q_cmd).round(3)
            tip_vel = (J @ dq).round(3)
            self.get_logger().info(
                'KDL='
                f'{has_kdl} | command_frame={self._command_frame} | '
                f'v_tool={cart_vel_tool.round(3)} | '
                f'v_base={cart_vel.round(3)} | tip_pos={tip_pos} | '
                f'tip_cmd={tip_cmd} | tip_vel={tip_vel} | '
                f'dq={dq.round(3)} | scale={scale:.3f}'
            )

        # ── Open-Loop Integration & Global limit scaling ─────────────────────
        # We integrate the commands directly into _q_cmd. This solves "Zeno's paradox"
        # of constantly resetting the JointTrajectoryController spline from current state.

        trajectory_duration = (
            self.get_parameter('trajectory_horizon_sec').get_parameter_value().double_value
        )
        trajectory_duration = max(trajectory_duration, dt * 1.5)

        limit_scale = 1.0
        for i in range(self.N_JOINTS):
            if dq[i] > 0.0:
                room = self._q_hi[i] - self._q_cmd[i]
                required = dq[i] * trajectory_duration
                if required > 0.0:
                    limit_scale = min(limit_scale, max(0.0, room / required))
            elif dq[i] < 0.0:
                room = self._q_cmd[i] - self._q_lo[i]
                required = abs(dq[i]) * trajectory_duration
                if required > 0.0:
                    limit_scale = min(limit_scale, max(0.0, room / required))

        if limit_scale < 1.0:
            dq = dq * limit_scale

        for i in range(self.N_JOINTS):
            self._q_cmd[i] += dq[i] * dt
            self._q_cmd[i] = float(np.clip(self._q_cmd[i], self._q_lo[i], self._q_hi[i]))

        if self._command_mode == 'velocity':
            self._publish_velocity(dq.tolist())
        else:
            # Push the command state forward and publish it as the target.
            self._publish_traj(self._q_cmd.tolist(), dq.tolist(), trajectory_duration)

    # ── Trajectory publisher ──────────────────────────────────────────────────

    def _publish_traj(self, positions: list[float], velocities: list[float], duration_sec: float):
        traj = JointTrajectory()
        # A stamp of 0 indicates execution immediately (overriding existing commands properly)
        traj.header.stamp.sec = 0
        traj.header.stamp.nanosec = 0
        traj.header.frame_id = self._command_frame
        traj.joint_names = self.JOINT_NAMES

        # Target point with explicit velocities
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.velocities = velocities
        pt.time_from_start = Duration(nanoseconds=int(duration_sec * 1e9)).to_msg()

        # Stop point to satisfy JTC constraint (prevent "last trajectory point is not zero" error)
        # JTC generally requires the final trajectory point to have a velocity of 0.0.
        # So we add a dummy stop point further out. Because this loop runs at 33 Hz,
        # this point is constantly preempted and never physically reached during continuous motion.
        pt_stop = JointTrajectoryPoint()

        # Extrapolate slightly further to maintain smooth braking curve if command stops
        stop_duration = duration_sec + 0.25
        stop_positions = [0.0] * self.N_JOINTS
        for i in range(self.N_JOINTS):
            # predict where it would stop
            stop_pos = positions[i] + velocities[i] * 0.1
            stop_pos = float(np.clip(stop_pos, self._q_lo[i], self._q_hi[i]))
            stop_positions[i] = stop_pos

        pt_stop.positions = stop_positions
        pt_stop.velocities = [0.0] * self.N_JOINTS
        pt_stop.time_from_start = Duration(nanoseconds=int(stop_duration * 1e9)).to_msg()

        traj.points = [pt, pt_stop]
        self._traj_pub.publish(traj)

    def _publish_velocity(self, velocities: list[float]):
        msg = Float64MultiArray()
        msg.data = velocities
        self._vel_pub.publish(msg)

    # ── Services ──────────────────────────────────────────────────────────────

    def _srv_home(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        """Send arm to home position (all zeros)."""
        dur = self.get_parameter('home_duration_sec').get_parameter_value().double_value
        self._publish_traj(self.HOME_POSITION, [0.0] * self.N_JOINTS, dur)
        res.success = True
        res.message = f'Moving to home position over {dur:.1f}s'
        self.get_logger().info(res.message)
        return res

    def _srv_switch_vel(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        """Switch between LINEAR (True) and ANGULAR (False) Cartesian velocity."""
        self._linear_mode = req.data
        res.success = True
        res.message = 'Velocity type → LINEAR' if req.data else 'Velocity type → ANGULAR'
        self.get_logger().info(res.message)
        return res

    def _srv_set_mode(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        """Switch between VELOCITY mode (True) and POSITION-INCREMENT mode (False)."""
        self._velocity_mode = req.data
        res.success = True
        res.message = 'Mode → VELOCITY (hold to move)' if req.data else 'Mode → POSITION INCREMENT'
        self.get_logger().info(res.message)
        return res

    def _srv_beak(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        """Close (True) or open (False) the beak gripper."""
        self._beak_pub.publish(Int32(data=int(req.data)))
        res.success = True
        res.message = f'Beak → {"CLOSE" if req.data else "OPEN"}'
        self.get_logger().info(res.message)
        return res


# ── Entry point ───────────────────────────────────────────────────────────────


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CartesianArmController()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as err:
        rclpy.logging.get_logger('cartesian_arm_controller').fatal(
            f'Unhandled exception: {err}\n{traceback.format_exc()}'
        )
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
