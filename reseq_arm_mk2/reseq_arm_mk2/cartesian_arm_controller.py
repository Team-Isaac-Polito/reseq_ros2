#!/usr/bin/env python3
"""
Cartesian arm controller for the RESE.Q MK2 arm.

It reads joint states, turns Cartesian velocity commands into joint motion
with a damped Jacobian solve, and publishes the result to the arm
controller. It also exposes a few small services for homing and mode
switching.
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
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Try to use KDL when it is available.
_HAS_KDL = False
_KDL_ERR = 'not attempted'
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


def _clamp_joint_velocity_to_limits(
    current_q: np.ndarray,
    dq: np.ndarray,
    q_lo: np.ndarray,
    q_hi: np.ndarray,
    dt: float,
) -> np.ndarray:
    """Clamp per-joint velocity so the next step cannot cross hard limits."""
    dq_limited = np.array(dq, dtype=float, copy=True)
    for i in range(len(dq_limited)):
        if dq_limited[i] > 0.0:
            remaining = max(0.0, q_hi[i] - current_q[i])
            dq_limited[i] = min(dq_limited[i], remaining / dt)
        elif dq_limited[i] < 0.0:
            remaining = max(0.0, current_q[i] - q_lo[i])
            dq_limited[i] = max(dq_limited[i], -remaining / dt)
    return dq_limited


def _compute_joint_hold_velocity(
    current_q: np.ndarray,
    target_q: np.ndarray,
    q_lo: np.ndarray,
    q_hi: np.ndarray,
    dt: float,
    gain: float,
    max_joint_vel: float,
    tolerance: float,
) -> np.ndarray:
    """Proportional pose hold in joint-velocity space around a target pose."""
    error = target_q - current_q
    dq = gain * error
    dq[np.abs(error) <= tolerance] = 0.0
    dq = np.clip(dq, -max_joint_vel, max_joint_vel)
    return _clamp_joint_velocity_to_limits(current_q, dq, q_lo, q_hi, dt)


def _advance_velocity_hold_target(
    current_q: np.ndarray,
    dq: np.ndarray,
    q_lo: np.ndarray,
    q_hi: np.ndarray,
    dt: float,
) -> np.ndarray:
    """Keep the velocity-mode hold target near the measured arm state.

    In velocity mode the robot only receives `dq`, not a future joint position
    target. Advancing the internal hold target by a long trajectory horizon
    makes the arm chase a phantom pose when the operator releases the stick.
    """
    return np.clip(current_q + dq * dt, q_lo, q_hi)


def _startup_recovery_command(
    current_q: np.ndarray,
    target_q: np.ndarray,
    q_lo: np.ndarray,
    q_hi: np.ndarray,
    dt: float,
    gain: float,
    max_joint_vel: float,
    tolerance: float,
) -> tuple[np.ndarray, bool]:
    """Return the startup hold velocity and whether the target pose is reached."""
    max_error = float(np.max(np.abs(target_q - current_q)))
    if max_error <= tolerance:
        return np.zeros_like(current_q), True
    dq = _compute_joint_hold_velocity(
        current_q=current_q,
        target_q=target_q,
        q_lo=q_lo,
        q_hi=q_hi,
        dt=dt,
        gain=gain,
        max_joint_vel=max_joint_vel,
        tolerance=tolerance,
    )
    return dq, False


def _unwrap_joint_positions(
    current_q: np.ndarray,
    reference_q: np.ndarray,
    q_lo: np.ndarray,
    q_hi: np.ndarray,
) -> np.ndarray:
    """Keep periodic joints continuous by unwrapping them against a reference pose."""
    unwrapped_q = np.array(current_q, dtype=float, copy=True)
    periodic_mask = (q_hi - q_lo) >= (2.0 * np.pi - 0.05)
    if not np.any(periodic_mask):
        return unwrapped_q

    delta = unwrapped_q[periodic_mask] - reference_q[periodic_mask]
    unwrapped_q[periodic_mask] = (
        reference_q[periodic_mask] + ((delta + np.pi) % (2.0 * np.pi)) - np.pi
    )
    return unwrapped_q


def _solve_weighted_dls_task_velocity(
    task_jacobian: np.ndarray,
    cart_vel: np.ndarray,
    joint_weights: np.ndarray,
    damping: float,
) -> np.ndarray:
    """Solve the weighted damped least-squares task velocity for the active joints."""
    inv_joint_weights = np.diag(1.0 / joint_weights)
    jj_t = task_jacobian @ inv_joint_weights @ task_jacobian.T
    return (
        inv_joint_weights
        @ task_jacobian.T
        @ np.linalg.solve(
            jj_t + damping**2 * np.eye(task_jacobian.shape[0]),
            cart_vel,
        )
    )


def _solve_task_velocity_with_limit_redistribution(
    current_q: np.ndarray,
    task_jacobian: np.ndarray,
    cart_vel: np.ndarray,
    q_lo: np.ndarray,
    q_hi: np.ndarray,
    dt: float,
    max_joint_vel: float,
    damping: float,
    joint_weights: np.ndarray,
) -> tuple[np.ndarray, list[str], float]:
    """Solve a task velocity and re-run it when a joint clips against a limit.

    If one joint saturates, remove it from the current solve and let the remaining
    joints redistribute the same Cartesian request instead of freezing the whole arm.
    """
    n_joints = task_jacobian.shape[1]
    active = np.ones(n_joints, dtype=bool)
    clipped_joints: list[str] = []
    dq_final = np.zeros(n_joints)
    vel_scale = 1.0

    for _ in range(n_joints):
        active_indices = np.flatnonzero(active)
        if active_indices.size == 0:
            break

        active_jacobian = task_jacobian[:, active_indices]
        active_weights = np.array(joint_weights[active_indices], dtype=float)
        dq_active = _solve_weighted_dls_task_velocity(
            task_jacobian=active_jacobian,
            cart_vel=cart_vel,
            joint_weights=active_weights,
            damping=damping,
        )

        dq_candidate = np.zeros(n_joints)
        dq_candidate[active_indices] = dq_active
        dq_candidate += _joint_limit_recovery_velocity(
            current_q=current_q,
            q_lo=q_lo,
            q_hi=q_hi,
            max_joint_vel=max_joint_vel,
        )

        peak = float(np.max(np.abs(dq_active)))
        vel_scale = 1.0
        if peak > max_joint_vel:
            vel_scale = max_joint_vel / peak
            dq_candidate[active_indices] *= vel_scale

        dq_clamped = _clamp_joint_velocity_to_limits(current_q, dq_candidate, q_lo, q_hi, dt)
        clipped_indices = []
        for idx in active_indices:
            if dq_candidate[idx] > 0.0 and dq_clamped[idx] < dq_candidate[idx]:
                clipped_indices.append(idx)
            elif dq_candidate[idx] < 0.0 and dq_clamped[idx] > dq_candidate[idx]:
                clipped_indices.append(idx)

        dq_final = dq_clamped
        if not clipped_indices:
            return dq_final, clipped_joints, vel_scale

        for idx in clipped_indices:
            active[idx] = False
            clipped_joints.append(f'J{idx}{"↑" if dq_candidate[idx] > 0.0 else "↓"}')

    return dq_final, clipped_joints, vel_scale


def _joint_limit_hold_scale(
    current_q: np.ndarray,
    q_lo: np.ndarray,
    q_hi: np.ndarray,
    margin_ratio: float = 0.15,
) -> float:
    """Fade the idle z hold as any joint approaches its hard limit."""
    span = np.maximum(q_hi - q_lo, 1e-6)
    limit_margin = np.maximum(span * margin_ratio, 1e-6)
    distance_to_limit = np.minimum(current_q - q_lo, q_hi - current_q)
    normalized_distance = np.clip(distance_to_limit / limit_margin, 0.0, 1.0)
    hold_scale = float(np.min(normalized_distance))
    return hold_scale * hold_scale


def _joint_limit_recovery_velocity(
    current_q: np.ndarray,
    q_lo: np.ndarray,
    q_hi: np.ndarray,
    max_joint_vel: float,
    margin_ratio: float = 0.2,
    recovery_gain: float = 0.25,
) -> np.ndarray:
    """Nudge joints back toward the interior when the arm is near a hard stop."""
    span = np.maximum(q_hi - q_lo, 1e-6)
    limit_margin = np.maximum(span * margin_ratio, 1e-6)
    distance_to_limit = np.minimum(current_q - q_lo, q_hi - current_q)
    proximity = np.clip((limit_margin - distance_to_limit) / limit_margin, 0.0, 1.0)
    if not np.any(proximity):
        return np.zeros_like(current_q)

    center = 0.5 * (q_lo + q_hi)
    recovery = np.clip((center - current_q) / limit_margin, -1.0, 1.0)
    recovery *= proximity * recovery_gain * max_joint_vel
    return recovery


class CartesianArmController(Node):
    """
    Jacobian-based Cartesian arm controller.

    Subscribed Topics
    -----------------
    /mk2_arm_vel          geometry_msgs/Vector3   joystick input  [-1, 1]
    /arm_joint_states     sensor_msgs/JointState

    Published Topics
    ----------------
    /mk2_arm_controller/joint_trajectory          trajectory_msgs/JointTrajectory
    /joint_group_velocity_controller/commands     std_msgs/Float64MultiArray

    Services
    --------
    /cartesian_arm_controller/go_home       std_srvs/Trigger
    /cartesian_arm_controller/switch_vel    std_srvs/SetBool  True=linear False=angular
    /cartesian_arm_controller/set_mode      std_srvs/SetBool  True=velocity False=pos-incr

    Parameters
    ----------
    robot_description      str    URDF XML (forwarded from robot_state_publisher)
    chain_root             str    'arm_base_link'
    chain_tip              str    'tool0'
    command_frame          str    'arm_base_link'
    state_topic            str    '/arm_joint_states'
    trajectory_topic       str    '/mk2_arm_controller/joint_trajectory'
    control_rate           float  33.0   Hz
    max_cartesian_vel      float  0.3    m/s  (joystick [-1,1] scaled by this)
    max_joint_vel          float  1.0    rad/s per joint
    home_duration_sec      float  3.0    s
    trajectory_horizon_sec float  0.10   s
    command_mode           str    'trajectory' or 'velocity'
    deadzone               float  0.02
    jacobian_damping       float  0.05   damped-LS regularisation λ
    joint_weights          float[]  joint weighting for the IK solve
    """

    # Joint order used everywhere in this controller.

    JOINT_NAMES = [
        'mod1__base_pitch_arm_joint',  # J0 shoulder pitch  (differential pair)
        'mod1__base_roll_arm_joint',  # J1 shoulder roll   (differential pair)
        'mod1__elbow_pitch_arm_joint',  # J2 elbow pitch
        'mod1__forearm_roll_arm_joint',  # J3 forearm roll
        'mod1__wrist_pitch_arm_joint',  # J4 wrist pitch
        'mod1__wrist_roll_arm_joint',  # J5 wrist roll
    ]
    N_JOINTS = len(JOINT_NAMES)

    # Fallback home pose used until the startup pose is captured from joint states.
    HOME_POSITION = [0.8, 0.0, 0.8, 0.0, 0.0, 0.0]
    HOME_TOLERANCE = 0.03
    JOINT_WEIGHTS = np.ones(6, dtype=float)

    # Defaults – _parse_urdf_limits() replaces these from the actual URDF.
    _LOWER_DEFAULT = np.array([-0.10, -3.14, -0.10, -3.14, -0.46, -3.14])
    _UPPER_DEFAULT = np.array([2.80, 3.14, 2.88, 3.14, 1.57, 3.14])

    def __init__(self):
        super().__init__('cartesian_arm_controller')

        # Parameters we expect from the launch file.
        self.declare_parameter('robot_description', '')
        self.declare_parameter('chain_root', 'arm_base_link')
        self.declare_parameter('chain_tip', 'tool0')
        self.declare_parameter('command_frame', 'arm_base_link')
        self.declare_parameter('state_topic', '/arm_joint_states')
        self.declare_parameter('trajectory_topic', '/mk2_arm_controller/joint_trajectory')
        self.declare_parameter('control_rate', 33.0)
        self.declare_parameter('max_cartesian_vel', 0.6)
        self.declare_parameter('max_joint_vel', 0.5)
        self.declare_parameter('home_duration_sec', 3.0)
        self.declare_parameter('trajectory_horizon_sec', 0.10)
        self.declare_parameter('command_mode', 'trajectory')
        self.declare_parameter('startup_hold_positions', [float('nan')] * self.N_JOINTS)
        self.declare_parameter('deadzone', 0.02)
        self.declare_parameter('jacobian_damping', 0.05)
        self.declare_parameter('joint_weights', self.JOINT_WEIGHTS.tolist())
        self.declare_parameter('idle_hold_gain', 1.5)
        self.declare_parameter('idle_hold_tolerance', 0.01)
        self.declare_parameter('front_branch_gain', 0.05)

        # Internal state.
        self._q: np.ndarray | None = None  # measured joint positions
        self._q_continuous: np.ndarray | None = None  # unwraps periodic joints across ±pi
        self._q_cmd: np.ndarray | None = None  # integrator state
        self._cmd_vel = np.zeros(3)  # latest joystick command
        self._linear_mode = True
        self._velocity_mode = True
        self._moving = False
        self._ee_z_ref: float | None = None
        self._front_x_ref: float | None = None
        self._front_branch_gain = (
            self.get_parameter('front_branch_gain').get_parameter_value().double_value
        )
        self._diag_ctr = 0
        self._home_active = False
        self._startup_home: np.ndarray | None = None
        self._startup_measured_pose: np.ndarray | None = None
        self._startup_hold_sent = False
        self._startup_hold_complete = False
        self._startup_recovery_active = False
        self._startup_recovery_logged = False
        startup_hold_positions = list(self.get_parameter('startup_hold_positions').value)
        self._startup_hold_target = None
        if len(startup_hold_positions) == self.N_JOINTS:
            startup_hold_array = np.array(startup_hold_positions, dtype=float)
            if np.isfinite(startup_hold_array).any():
                self._startup_hold_target = startup_hold_array
        if self._startup_hold_target is None:
            self._startup_hold_complete = True

        joint_weights_param = self.get_parameter('joint_weights').value
        if isinstance(joint_weights_param, (list, tuple, np.ndarray)) and (
            len(joint_weights_param) == self.N_JOINTS
        ):
            self._joint_weights = np.array(joint_weights_param, dtype=float)
        else:
            self._joint_weights = self.JOINT_WEIGHTS.copy()

        # Joint limits, replaced later with values from the URDF.
        self._q_lo = self._LOWER_DEFAULT.copy()
        self._q_hi = self._UPPER_DEFAULT.copy()

        # KDL setup.
        self._kdl_chain = None
        self._fk_solver = None
        self._jac_solver = None
        self._chain_root = self.get_parameter('chain_root').get_parameter_value().string_value
        self._chain_tip = self.get_parameter('chain_tip').get_parameter_value().string_value
        self._load_kdl()

        self._command_frame = (
            self.get_parameter('command_frame').get_parameter_value().string_value
        )
        self._command_mode = self.get_parameter('command_mode').get_parameter_value().string_value
        state_topic = self.get_parameter('state_topic').get_parameter_value().string_value

        # ROS interfaces.
        self.create_subscription(Vector3, '/mk2_arm_vel', self._cb_vel, 10)
        self.create_subscription(JointState, state_topic, self._cb_joint_state, 10)

        traj_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value
        self._traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)
        self._vel_pub = self.create_publisher(
            Float64MultiArray, '/joint_group_velocity_controller/commands', 10
        )
        self._vel_pub_legacy = self.create_publisher(
            Float64MultiArray, '/joint_group_velocity_controller/command', 10
        )

        self.create_service(Trigger, '/cartesian_arm_controller/go_home', self._srv_home)
        self.create_service(SetBool, '/cartesian_arm_controller/switch_vel', self._srv_switch_vel)
        self.create_service(SetBool, '/cartesian_arm_controller/set_mode', self._srv_set_mode)

        rate = self.get_parameter('control_rate').get_parameter_value().double_value
        self._dt = 1.0 / rate
        self._timer = self.create_timer(self._dt, self._control_loop)

        kdl_status = 'YES' if self._kdl_chain else 'NO – numerical fallback'
        self.get_logger().info(
            f'CartesianArmController ready | KDL={kdl_status} | '
            f'frame={self._command_frame} | mode={self._command_mode} | '
            f'home={self.HOME_POSITION}'
        )

    # KDL setup.

    def _load_kdl(self):
        if not _HAS_KDL:
            self.get_logger().warn(
                f'KDL unavailable ({_KDL_ERR}). '
                'Using numerical Jacobian – Y-axis direction may be wrong.'
            )
            return

        urdf = self.get_parameter('robot_description').get_parameter_value().string_value
        if not urdf:
            self.get_logger().warn(
                'robot_description is empty -> KDL disabled. '
                'Pass robot_description parameter from the launch file.'
            )
            return

        try:
            ok, tree = treeFromString(urdf)
        except Exception as e:
            self.get_logger().error(f'treeFromString raised: {e}')
            return

        if not ok:
            self.get_logger().error('treeFromString failed – URDF may be malformed.')
            return

        root = self._chain_root
        tip = self._chain_tip
        chain = kdl.Chain()
        if not tree.getChain(root, tip, chain):
            fallback_tip = 'tool0'
            if tip != fallback_tip and tree.getChain(root, fallback_tip, chain):
                self.get_logger().warn(
                    f'KDL getChain({root} -> {tip}) failed; using {fallback_tip} instead.'
                )
                tip = fallback_tip
                self._chain_tip = fallback_tip
            else:
                fallback_tip = 'arm_roll_wrist_link'
                if tip != fallback_tip and tree.getChain(root, fallback_tip, chain):
                    self.get_logger().warn(
                        f'KDL getChain({root} -> {tip}) failed; using {fallback_tip} instead.'
                    )
                    tip = fallback_tip
                    self._chain_tip = fallback_tip
                else:
                    self.get_logger().error(f'KDL getChain({root} -> {tip}) failed.')
                    return

        n = chain.getNrOfJoints()
        if n != self.N_JOINTS:
            self.get_logger().warn(
                f'KDL chain has {n} joints, expected {self.N_JOINTS}. '
                'Check chain_root/chain_tip parameters.'
            )

        self._kdl_chain = chain
        self._fk_solver = kdl.ChainFkSolverPos_recursive(chain)
        self._jac_solver = kdl.ChainJntToJacSolver(chain)
        self._parse_urdf_limits(urdf)

        # Startup diagnostics.
        self.get_logger().info(
            f'KDL chain {root}->{tip}: {n} joint(s)\n'
            f'  lower: {np.round(self._q_lo, 3).tolist()}\n'
            f'  upper: {np.round(self._q_hi, 3).tolist()}'
        )

        q_home = np.array(self.HOME_POSITION, dtype=float)
        p_home = self._fk_kdl(q_home)
        if p_home is not None:
            self.get_logger().info(
                f'FK(home) = [{p_home[0]:.4f}, {p_home[1]:.4f}, {p_home[2]:.4f}] m'
            )
            self._front_x_ref = float(p_home[0])

        J_home = self._jacobian_kdl(q_home)
        if J_home is not None:
            rank = int(np.linalg.matrix_rank(J_home, tol=1e-3))
            self.get_logger().info(
                f'Jacobian at home position (3×{n}, rank={rank}):\n{np.round(J_home, 4)}'
            )
            if rank < 3:
                self.get_logger().warn(
                    f'Jacobian rank {rank} < 3 at home – arm is in a singular configuration! '
                    'Some Cartesian directions will not be controllable from home.'
                )

    def _parse_urdf_limits(self, urdf: str):
        """Read joint limits from the URDF."""
        try:
            root_el = ET.fromstring(urdf)
            for jel in root_el.findall('joint'):
                name = jel.get('name', '')
                if name not in self.JOINT_NAMES:
                    continue
                idx = self.JOINT_NAMES.index(name)
                lim = jel.find('limit')
                if lim is None:
                    continue
                lo = lim.get('lower')
                hi = lim.get('upper')
                if lo is not None:
                    self._q_lo[idx] = float(lo)
                if hi is not None:
                    self._q_hi[idx] = float(hi)
        except Exception as e:
            self.get_logger().warn(f'Could not parse URDF joint limits: {e}')

    # ROS callbacks.

    def _cb_joint_state(self, msg: JointState):
        pos_map = dict(zip(msg.name, msg.position))
        try:
            self._q = np.array([pos_map[n] for n in self.JOINT_NAMES])
            if self._q_continuous is None:
                self._q_continuous = self._q.copy()
            else:
                self._q_continuous = _unwrap_joint_positions(
                    current_q=self._q,
                    reference_q=self._q_continuous,
                    q_lo=self._q_lo,
                    q_hi=self._q_hi,
                )
            if self._startup_home is None:
                self._startup_measured_pose = self._q.copy()
                hold_tolerance = (
                    self.get_parameter('idle_hold_tolerance').get_parameter_value().double_value
                )
                if self._startup_hold_target is not None:
                    self._startup_home = np.clip(self._startup_hold_target, self._q_lo, self._q_hi)
                    startup_error = float(
                        np.max(np.abs(self._startup_home - self._startup_measured_pose))
                    )
                    self._startup_recovery_active = startup_error > hold_tolerance
                    self._startup_hold_complete = not self._startup_recovery_active
                    self.get_logger().info(
                        'Captured startup joint state: '
                        f'measured={np.round(self._startup_measured_pose, 3).tolist()} '
                        f'hold_target={np.round(self._startup_home, 3).tolist()}'
                    )
                else:
                    self._startup_home = self._q.copy()
                    self._startup_hold_complete = True
                    self.get_logger().info(
                        f'Captured startup home pose: {np.round(self._startup_home, 3).tolist()}'
                    )
        except KeyError:
            pass  # not all arm joints present yet

    def _cb_vel(self, msg: Vector3):
        self._cmd_vel = np.array([msg.x, msg.y, msg.z])

    # Forward kinematics.

    def _fk_kdl(self, q: np.ndarray) -> np.ndarray | None:
        n = self._kdl_chain.getNrOfJoints()
        q_kdl = kdl.JntArray(n)
        for i in range(min(n, self.N_JOINTS)):
            q_kdl[i] = float(q[i])
        frame = kdl.Frame()
        if self._fk_solver.JntToCart(q_kdl, frame) < 0:
            return None
        return np.array([frame.p.x(), frame.p.y(), frame.p.z()])

    def _fk_frame_kdl(self, q: np.ndarray) -> kdl.Frame | None:
        n = self._kdl_chain.getNrOfJoints()
        q_kdl = kdl.JntArray(n)
        for i in range(min(n, self.N_JOINTS)):
            q_kdl[i] = float(q[i])

        frame = kdl.Frame()
        if self._fk_solver.JntToCart(q_kdl, frame) < 0:
            return None
        return frame

    def _fk_numerical(self, q: np.ndarray) -> np.ndarray:
        """
        Simple FK fallback for cases where KDL is not available.

        It uses joint origin translations from the URDF, so it is only an
        approximation and will not capture the full joint orientation math.
        """
        # Approximate link lengths taken from the URDF joint origins.
        L1, L2, L3 = 0.289, 0.170, 0.054
        q0, q1, q2, _, q4, _ = q
        az = q1  # azimuth (base_roll)
        el0 = q0  # shoulder elevation
        el2 = q0 + q2  # elbow elevation
        el4 = q0 + q2 + q4
        reach = L1 * np.cos(el0) + L2 * np.cos(el2) + L3 * np.cos(el4)
        x = np.cos(az) * reach
        y = np.sin(az) * reach
        z = L1 * np.sin(el0) + L2 * np.sin(el2) + L3 * np.sin(el4)
        return np.array([x, y, z])

    def _get_ee_pos(self, q: np.ndarray) -> np.ndarray:
        if self._fk_solver is not None:
            r = self._fk_kdl(q)
            if r is not None:
                return r
        return self._fk_numerical(q)

    def _command_velocity_in_base(self, q: np.ndarray, cmd_vel: np.ndarray) -> np.ndarray:
        command_frame = self._command_frame
        if command_frame == self._chain_root:
            return cmd_vel

        if self._fk_solver is None:
            self.get_logger().warn(
                f'Cannot transform commands from {command_frame} without KDL; using base frame.',
                throttle_duration_sec=5.0,
            )
            return cmd_vel

        if command_frame != self._chain_tip:
            self.get_logger().warn(
                f'Unsupported command_frame {command_frame}; using base frame command.',
                throttle_duration_sec=5.0,
            )
            return cmd_vel

        frame = self._fk_frame_kdl(q)
        if frame is None:
            return cmd_vel

        base_vec = frame.M * kdl.Vector(float(cmd_vel[0]), float(cmd_vel[1]), float(cmd_vel[2]))
        return np.array([base_vec[0], base_vec[1], base_vec[2]])

    # Jacobian.

    def _jacobian_kdl(self, q: np.ndarray) -> np.ndarray | None:
        """
        Return the full 6D Jacobian via KDL.

        Rows 0-2 are linear velocity and rows 3-5 are angular velocity.
        The element-wise access is used because it works reliably across
        PyKDL builds.
        """
        n = self._kdl_chain.getNrOfJoints()
        q_kdl = kdl.JntArray(n)
        for i in range(min(n, self.N_JOINTS)):
            q_kdl[i] = float(q[i])

        jac = kdl.Jacobian(n)
        if self._jac_solver.JntToJac(q_kdl, jac) < 0:
            self.get_logger().warn('KDL JntToJac error.')
            return None

        try:
            J = np.array([[jac[r, c] for c in range(n)] for r in range(6)])
        except Exception as e:
            self.get_logger().warn(f'Jacobian extraction failed ({e}) – falling back.')
            return None

        return J

    def _jacobian_numerical(self, q: np.ndarray, eps: float = 1e-4) -> np.ndarray:
        p0 = self._fk_numerical(q)
        J = np.zeros((6, self.N_JOINTS))
        for i in range(self.N_JOINTS):
            dq = q.copy()
            dq[i] += eps
            J[:3, i] = (self._fk_numerical(dq) - p0) / eps
        return J

    def _get_jacobian(self, q: np.ndarray) -> np.ndarray | None:
        if self._jac_solver is not None:
            return self._jacobian_kdl(q)
        self.get_logger().warn(
            'KDL not available – numerical Jacobian (Y-axis may be wrong!)',
            throttle_duration_sec=5.0,
        )
        return self._jacobian_numerical(q)

    # Main control loop (33 Hz).

    def _control_loop(self):
        if self._q is None:
            return

        # Initialize the integrator on the first valid joint state.
        if self._q_cmd is None:
            self._q_cmd = (
                self._startup_home.copy() if self._startup_home is not None else self._q.copy()
            )
            self._moving = False

        if self._command_mode == 'trajectory' and not self._startup_hold_sent:
            if self._traj_pub.get_subscription_count() > 0:
                startup_target = self._q_cmd.copy() if self._q_cmd is not None else self._q.copy()
                self._q_cmd = startup_target.copy()
                self._publish_traj(self._q.tolist(), startup_target.tolist(), 0.1)
                self._startup_hold_sent = True

        deadzone = self.get_parameter('deadzone').get_parameter_value().double_value
        cmd_norm = float(np.linalg.norm(self._cmd_vel))
        # Treat a perfectly centered stick as idle even when deadzone is configured to 0.0.
        idle_threshold = max(deadzone, 1e-6)

        if self._home_active:
            self._run_home_velocity()
            return

        if self._startup_recovery_active:
            self._run_startup_recovery()
            return

        # Ignore tiny inputs.
        if cmd_norm <= idle_threshold:
            if self._command_mode == 'velocity':
                max_jv = self.get_parameter('max_joint_vel').get_parameter_value().double_value
                hold_gain = self.get_parameter('idle_hold_gain').get_parameter_value().double_value
                hold_tolerance = (
                    self.get_parameter('idle_hold_tolerance').get_parameter_value().double_value
                )
                hold_target = self._q_cmd.copy() if self._q_cmd is not None else self._q.copy()
                if not self._startup_hold_complete and self._startup_hold_target is not None:
                    startup_target = np.clip(self._startup_hold_target, self._q_lo, self._q_hi)
                    hold_target = startup_target
                    if float(np.max(np.abs(startup_target - self._q))) <= hold_tolerance:
                        self._startup_hold_complete = True
                hold_dq = _compute_joint_hold_velocity(
                    current_q=self._q,
                    target_q=hold_target,
                    q_lo=self._q_lo,
                    q_hi=self._q_hi,
                    dt=self._dt,
                    gain=hold_gain,
                    max_joint_vel=max_jv,
                    tolerance=hold_tolerance,
                )
                self._q_cmd = hold_target.copy()
                self._publish_velocity(hold_dq.tolist())
            else:
                # In trajectory mode the arm needs a steady hold target while idle.
                # Without this, Gazebo can let the joints settle under gravity after
                # the last short trajectory finishes, which shows up as a startup tilt.
                hold_target = self._q_cmd.tolist() if self._q_cmd is not None else self._q.tolist()
                self._publish_traj(self._q.tolist(), hold_target, 0.1)
            self._moving = False
            self._ee_z_ref = None
            return

        # Solve from the live measured pose so the Jacobian tracks the actual arm.
        if not self._moving:
            self._moving = True
        solve_q = self._q_continuous if self._q_continuous is not None else self._q
        if self._ee_z_ref is None:
            self._ee_z_ref = float(self._get_ee_pos(solve_q)[2])

        if not self._linear_mode:
            # Orientation mode is not implemented yet.
            return

        # Control parameters.
        max_cv = self.get_parameter('max_cartesian_vel').get_parameter_value().double_value
        max_jv = self.get_parameter('max_joint_vel').get_parameter_value().double_value
        lam = self.get_parameter('jacobian_damping').get_parameter_value().double_value
        horizon = self.get_parameter('trajectory_horizon_sec').get_parameter_value().double_value
        horizon = max(horizon, self._dt * 2.0)  # never shorter than 2 control ticks

        # Interpret the input in the configured command frame, then solve in base coordinates.
        cart_vel_cmd = self._cmd_vel * max_cv
        cart_vel = self._command_velocity_in_base(solve_q, cart_vel_cmd)

        # Hold the current height unless the user is explicitly commanding Z.
        # This keeps lateral motion from slowly climbing as the arm changes posture.
        if self._ee_z_ref is not None and abs(cart_vel_cmd[2]) < deadzone:
            z_error = self._ee_z_ref - float(self._get_ee_pos(solve_q)[2])
            z_hold_vel = 2.0 * z_error
            hold_scale = _joint_limit_hold_scale(solve_q, self._q_lo, self._q_hi)
            # Relax the height hold near saturation so a reverse x/y command can
            # back the arm away from the limit instead of fighting the stale z target.
            cart_vel[2] = float(np.clip(z_hold_vel * hold_scale, -max_cv, max_cv))

        if self._front_x_ref is not None and abs(cart_vel_cmd[0]) < deadzone:
            x_error = self._front_x_ref - float(self._get_ee_pos(solve_q)[0])
            if x_error > 0.0:
                # Keep lateral motion on the front side of the workspace unless the
                # operator is explicitly commanding X.
                cart_vel[0] = float(np.clip(2.0 * x_error, 0.0, max_cv))

        J = self._get_jacobian(solve_q)
        if J is None:
            return

        # Damped least-squares IK on the tool translational task.
        try:
            active_dofs = J.shape[1]
            Jlin = J[:3, :active_dofs]
            joint_weights = self._joint_weights[:active_dofs]
            dq, joints_clipped, vel_scale = _solve_task_velocity_with_limit_redistribution(
                current_q=solve_q,
                task_jacobian=Jlin,
                cart_vel=cart_vel,
                q_lo=self._q_lo,
                q_hi=self._q_hi,
                dt=self._dt,
                max_joint_vel=max_jv,
                damping=lam,
                joint_weights=joint_weights,
            )
        except np.linalg.LinAlgError:
            self.get_logger().warn('DLS solve failed.')
            return

        if (
            self._startup_home is not None
            and self._front_x_ref is not None
            and abs(cart_vel_cmd[0]) < deadzone
            and self._front_branch_gain > 0.0
        ):
            # Bias the nullspace toward the startup/front branch while the user is
            # only steering y/z. This keeps reversals from flipping to the back side.
            branch_bias = np.clip(
                self._front_branch_gain * (self._startup_home - solve_q),
                -0.25 * max_jv,
                0.25 * max_jv,
            )
            dq = dq + branch_bias
            dq = np.clip(dq, -max_jv, max_jv)
            dq = _clamp_joint_velocity_to_limits(solve_q, dq, self._q_lo, self._q_hi, self._dt)

        # In trajectory mode, generate the next target from the measured pose.
        # This keeps the published joint step consistent with the Jacobian
        # linearization and avoids accumulating backlog when hardware tracking
        # lags behind the previously commanded target.
        if self._command_mode == 'trajectory':
            self._q_cmd = np.clip(solve_q + dq * horizon, self._q_lo, self._q_hi)
        else:
            # Velocity mode should hold near the actual arm pose, not a
            # long-horizon prediction. Using the trajectory lookahead here
            # causes a snap when manual input returns to zero.
            self._q_cmd = _advance_velocity_hold_target(
                current_q=solve_q,
                dq=dq,
                q_lo=self._q_lo,
                q_hi=self._q_hi,
                dt=self._dt,
            )

        # Print diagnostics at 1 Hz.
        self._diag_ctr += 1
        if self._diag_ctr % 33 == 0:
            ee = self._get_ee_pos(self._q_cmd if self._q_cmd is not None else self._q)
            achieved_cart = Jlin @ dq[:active_dofs]
            self.get_logger().info(
                f'cart_in={np.round(cart_vel, 3)} '
                f'dq={np.round(dq, 3)} '
                f'cart_out={np.round(achieved_cart, 3)} '
                f'vel_scale={vel_scale:.2f} '
                f'clipped={joints_clipped} '
                f'vel_subs={self._vel_pub.get_subscription_count()} '
                f'vel_legacy_subs={self._vel_pub_legacy.get_subscription_count()}\n'
                f'  q_cmd={np.round(self._q_cmd, 3)}\n'
                f'  q_meas={np.round(self._q, 3)}\n'
                f'  EE={np.round(ee, 4)} m'
            )

        # Publish the command.
        if self._command_mode == 'velocity':
            self._publish_velocity(dq.tolist())
        else:
            self._publish_traj(self._q.tolist(), self._q_cmd.tolist(), horizon)

    def _run_home_velocity(self):
        """Drive the arm toward the configured home target in velocity mode."""
        if self._q is None:
            return

        target = self._get_home_position()
        self._q_cmd = target.copy()
        current_q = self._q_continuous if self._q_continuous is not None else self._q
        error = target - current_q

        if float(np.max(np.abs(error))) < self.HOME_TOLERANCE:
            self._home_active = False
            self._moving = False
            self._ee_z_ref = None
            self._publish_velocity([0.0] * self.N_JOINTS)
            self.get_logger().info('Home reached.')
            return

        max_jv = self.get_parameter('max_joint_vel').get_parameter_value().double_value
        dur = self.get_parameter('home_duration_sec').get_parameter_value().double_value
        home_gain = 1.0 / max(dur, self._dt)
        dq = error * home_gain
        dq = np.clip(dq, -max_jv, max_jv)

        dq = _clamp_joint_velocity_to_limits(self._q, dq, self._q_lo, self._q_hi, self._dt)

        self._publish_velocity(dq.tolist())

    def _run_startup_recovery(self):
        """Recover the arm to the configured startup hold pose before accepting manual motion."""
        if self._q is None or self._startup_home is None:
            return

        max_jv = self.get_parameter('max_joint_vel').get_parameter_value().double_value
        hold_gain = self.get_parameter('idle_hold_gain').get_parameter_value().double_value
        hold_tolerance = (
            self.get_parameter('idle_hold_tolerance').get_parameter_value().double_value
        )
        current_q = self._q_continuous if self._q_continuous is not None else self._q
        dq, reached_target = _startup_recovery_command(
            current_q=current_q,
            target_q=self._startup_home,
            q_lo=self._q_lo,
            q_hi=self._q_hi,
            dt=self._dt,
            gain=hold_gain,
            max_joint_vel=max_jv,
            tolerance=hold_tolerance,
        )

        self._q_cmd = self._startup_home.copy()
        self._moving = False
        self._ee_z_ref = None

        if not self._startup_recovery_logged:
            self.get_logger().info(
                'Startup recovery active: '
                f'measured={np.round(self._q, 3).tolist()} '
                f'target={np.round(self._startup_home, 3).tolist()}'
            )
            self._startup_recovery_logged = True

        if reached_target:
            self._startup_recovery_active = False
            self._startup_hold_complete = True
            self._startup_recovery_logged = False
            self._publish_velocity([0.0] * self.N_JOINTS)
            self.get_logger().info('Startup recovery complete.')
            return

        self._publish_velocity(dq.tolist())

    def _get_home_position(self) -> np.ndarray:
        if self._startup_home is not None:
            return self._startup_home.copy()
        return np.array(self.HOME_POSITION, dtype=float)

    # Trajectory publisher.

    def _publish_traj(
        self, start_positions: list[float], target_positions: list[float], duration_sec: float
    ):
        """Publish a short two-point trajectory for the arm controller."""
        traj = JointTrajectory()
        traj.header.stamp.sec = 0  # execute immediately, preempt current
        traj.header.stamp.nanosec = 0
        traj.header.frame_id = self._command_frame
        traj.joint_names = self.JOINT_NAMES

        # Never send positions outside the joint limits.
        start_safe = [
            float(np.clip(start_positions[i], self._q_lo[i], self._q_hi[i]))
            for i in range(self.N_JOINTS)
        ]
        target_safe = [
            float(np.clip(target_positions[i], self._q_lo[i], self._q_hi[i]))
            for i in range(self.N_JOINTS)
        ]

        pt_start = JointTrajectoryPoint()
        pt_start.positions = start_safe
        pt_start.velocities = [0.0] * self.N_JOINTS
        pt_start.time_from_start = Duration(nanoseconds=0).to_msg()

        pt_target = JointTrajectoryPoint()
        pt_target.positions = target_safe
        pt_target.velocities = [0.0] * self.N_JOINTS
        pt_target.time_from_start = Duration(nanoseconds=int(duration_sec * 1e9)).to_msg()

        traj.points = [pt_start, pt_target]
        self._traj_pub.publish(traj)

    def _publish_velocity(self, velocities: list[float]):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in velocities]
        self._vel_pub.publish(msg)
        self._vel_pub_legacy.publish(msg)

    # Services.

    def _srv_home(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        dur = self.get_parameter('home_duration_sec').get_parameter_value().double_value
        if self._q is None:
            res.success = False
            res.message = 'No joint state yet; cannot home arm'
            self.get_logger().warn(res.message)
            return res

        home_target = self._get_home_position()
        self._q_cmd = home_target.copy()
        self._moving = True
        self._cmd_vel = np.zeros(3)
        self._ee_z_ref = None
        if self._command_mode == 'velocity':
            self._home_active = True
            res.success = True
            res.message = f'Homing to {np.round(home_target, 3).tolist()} over {dur:.1f}s'
        else:
            self._home_active = False
            start_positions = self._q.tolist() if self._q is not None else home_target.tolist()
            self._publish_traj(start_positions, home_target.tolist(), dur)
            res.success = True
            res.message = f'Moving to home {np.round(home_target, 3).tolist()} over {dur:.1f}s'
        self.get_logger().info(res.message)
        return res

    def _srv_switch_vel(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        self._linear_mode = req.data
        res.success = True
        res.message = 'Velocity -> LINEAR' if req.data else 'Velocity -> ANGULAR'
        self.get_logger().info(res.message)
        return res

    def _srv_set_mode(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        self._command_mode = 'velocity' if req.data else 'trajectory'
        res.success = True
        res.message = 'Mode -> VELOCITY' if req.data else 'Mode -> POSITION INCREMENT'
        self.get_logger().info(res.message)
        return res


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
