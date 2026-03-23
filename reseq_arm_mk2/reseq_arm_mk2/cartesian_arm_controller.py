#!/usr/bin/env python3
"""
Jacobian-based Cartesian arm controller for reseq MK2.

Replaces MoveIt Servo with a direct Jacobian velocity loop that:
  - Reads /joint_states at ~33 Hz
  - Converts joystick Cartesian velocity → joint velocities via damped J†
  - Applies PER-JOINT limit clamping (blocked joint zeroed, others continue)
  - Publishes to /mk2_arm_controller/joint_trajectory
  - Exposes services for home, beak, and mode switching

Bug-fixes vs. previous version
--------------------------------
BUG 1 (X axis broken at home):
  Old code computed a single global limit_scale across all joints.  At home
  (q=0), elbow_pitch (J2) has only 0.1 rad of negative travel.  An X command
  pushes J2 negative → limit_scale = 0.1/(4.0×0.5) = 0.05 → ALL axes get
  scaled to 5% of their command, so the arm barely moves.

  Fix: per-joint clamping.  When J2 is blocked at its lower limit, only J2's
  velocity is zeroed.  J0, J1, … continue at full speed.

BUG 2 (Y axis broken / wrong direction):
  Old code used jac.getColumn(c).vel.x() which is fragile across PyKDL builds.
  If getColumn fails, the numerical fallback _fk_numerical is used.  That model
  gives ∂y/∂q1 = +0.52, while the actual URDF kinematics gives −0.127.  The
  OPPOSITE SIGN means Y commands move the arm backwards or not at all.

  Fix: use jac[r, c] element-wise indexing (universally supported).

BUG 3 (JTC rejects trajectory):
  The stop-point was computed as position + velocity*0.1, which can exceed
  joint limits and cause JTC to reject the entire trajectory message.

  Fix: stop-point uses the same clipped positions with zero velocity.

BUG 4 (home position near singularity):
  HOME_POSITION = [0,0,0,0,0,0] places J0 and J2 at 0 rad, only 0.1 rad
  above their lower limit (−0.1 rad), creating a near-singular configuration
  where any motion toward the lower limit immediately saturates.

  Fix: HOME_POSITION = [0.8, 0.0, 0.8, 0.0, 0.0, 0.0] – mid-range.
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


class CartesianArmController(Node):
    """
    Jacobian-based Cartesian arm controller.

    Subscribed Topics
    -----------------
    /mk2_arm_vel          geometry_msgs/Vector3   joystick input  [-1, 1]
    /joint_states         sensor_msgs/JointState

    Published Topics
    ----------------
    /mk2_arm_controller/joint_trajectory          trajectory_msgs/JointTrajectory
    /joint_group_velocity_controller/commands     std_msgs/Float64MultiArray
    reseq/module33/mk2_arm/beak/setpoint          std_msgs/Int32

    Services
    --------
    /cartesian_arm_controller/go_home       std_srvs/Trigger
    /cartesian_arm_controller/switch_vel    std_srvs/SetBool  True=linear False=angular
    /cartesian_arm_controller/set_mode      std_srvs/SetBool  True=velocity False=pos-incr
    /cartesian_arm_controller/close_beak    std_srvs/SetBool  True=close False=open

    Parameters
    ----------
    robot_description      str    URDF XML (forwarded from robot_state_publisher)
    chain_root             str    'arm_base_link'
    chain_tip              str    'arm_roll_wrist_link'
    command_frame          str    'arm_base_link'
    trajectory_topic       str    '/mk2_arm_controller/joint_trajectory'
    control_rate           float  33.0   Hz
    max_cartesian_vel      float  0.3    m/s  (joystick [-1,1] scaled by this)
    max_joint_vel          float  1.0    rad/s per joint
    home_duration_sec      float  3.0    s
    trajectory_horizon_sec float  0.10   s
    command_mode           str    'trajectory' or 'velocity'
    deadzone               float  0.02
    jacobian_damping       float  0.05   damped-LS regularisation λ
    """

    # ── Constants ──────────────────────────────────────────────────────────────

    JOINT_NAMES = [
        'mod1__base_pitch_arm_joint',  # J0 shoulder pitch  (differential pair)
        'mod1__base_roll_arm_joint',  # J1 shoulder roll   (differential pair)
        'mod1__elbow_pitch_arm_joint',  # J2 elbow pitch
        'mod1__forearm_roll_arm_joint',  # J3 forearm roll
        'mod1__wrist_pitch_arm_joint',  # J4 wrist pitch
        'mod1__wrist_roll_arm_joint',  # J5 wrist roll
    ]
    N_JOINTS = len(JOINT_NAMES)

    # Home: well inside limits, away from the ±0.1 rad lower-limit of J0/J2.
    # J0 range [-0.1, 2.8], J2 range [-0.1, 2.88].  0.8 rad gives ~0.9 rad
    # of negative room vs the 0.1 rad at home=[0,0,...].
    HOME_POSITION = [0.8, 0.0, 0.8, 0.0, 0.0, 0.0]

    # Defaults – _parse_urdf_limits() replaces these from the actual URDF.
    _LOWER_DEFAULT = np.array([-0.10, -3.14, -0.10, -3.14, -0.46, -3.14])
    _UPPER_DEFAULT = np.array([2.80, 3.14, 2.88, 3.14, 1.57, 3.14])

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
        self.declare_parameter('trajectory_horizon_sec', 0.10)
        self.declare_parameter('command_mode', 'trajectory')
        self.declare_parameter('deadzone', 0.02)
        self.declare_parameter('jacobian_damping', 0.05)

        # ── Runtime state ─────────────────────────────────────────────────────
        self._q: np.ndarray | None = None  # measured joint positions
        self._q_cmd: np.ndarray | None = None  # integrator state
        self._cmd_vel = np.zeros(3)  # latest joystick command
        self._linear_mode = True
        self._velocity_mode = True
        self._moving = False
        self._diag_ctr = 0

        # ── Joint limits (overwritten from URDF) ──────────────────────────────
        self._q_lo = self._LOWER_DEFAULT.copy()
        self._q_hi = self._UPPER_DEFAULT.copy()

        # ── KDL ───────────────────────────────────────────────────────────────
        self._kdl_chain = None
        self._fk_solver = None
        self._jac_solver = None
        self._load_kdl()

        self._command_frame = (
            self.get_parameter('command_frame').get_parameter_value().string_value
        )
        self._command_mode = self.get_parameter('command_mode').get_parameter_value().string_value

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self.create_subscription(Vector3, '/mk2_arm_vel', self._cb_vel, 10)
        self.create_subscription(JointState, '/joint_states', self._cb_joint_state, 10)

        traj_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value
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
        self._dt = 1.0 / rate
        self._timer = self.create_timer(self._dt, self._control_loop)

        kdl_status = 'YES ✓' if self._kdl_chain else 'NO – numerical fallback ⚠'
        self.get_logger().info(
            f'CartesianArmController ready | KDL={kdl_status} | '
            f'frame={self._command_frame} | mode={self._command_mode} | '
            f'home={self.HOME_POSITION}'
        )

    # ── KDL setup ─────────────────────────────────────────────────────────────

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
                'robot_description is empty → KDL disabled. '
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

        root = self.get_parameter('chain_root').get_parameter_value().string_value
        tip = self.get_parameter('chain_tip').get_parameter_value().string_value
        chain = kdl.Chain()
        if not tree.getChain(root, tip, chain):
            self.get_logger().error(f'KDL getChain({root} → {tip}) failed.')
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

        # ── Startup diagnostics ───────────────────────────────────────────────
        self.get_logger().info(
            f'KDL chain {root}→{tip}: {n} joint(s)\n'
            f'  lower: {np.round(self._q_lo, 3).tolist()}\n'
            f'  upper: {np.round(self._q_hi, 3).tolist()}'
        )

        q_home = np.array(self.HOME_POSITION, dtype=float)
        p_home = self._fk_kdl(q_home)
        if p_home is not None:
            self.get_logger().info(
                f'FK(home) = [{p_home[0]:.4f}, {p_home[1]:.4f}, {p_home[2]:.4f}] m'
            )

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
        """Extract <limit lower= upper=> from the URDF for each arm joint."""
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

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _cb_joint_state(self, msg: JointState):
        pos_map = dict(zip(msg.name, msg.position))
        try:
            self._q = np.array([pos_map[n] for n in self.JOINT_NAMES])
        except KeyError:
            pass  # not all arm joints present yet

    def _cb_vel(self, msg: Vector3):
        self._cmd_vel = np.array([msg.x, msg.y, msg.z])

    # ── Forward kinematics ────────────────────────────────────────────────────

    def _fk_kdl(self, q: np.ndarray) -> np.ndarray | None:
        n = self._kdl_chain.getNrOfJoints()
        q_kdl = kdl.JntArray(n)
        for i in range(min(n, self.N_JOINTS)):
            q_kdl[i] = float(q[i])
        frame = kdl.Frame()
        if self._fk_solver.JntToCart(q_kdl, frame) < 0:
            return None
        return np.array([frame.p.x(), frame.p.y(), frame.p.z()])

    def _fk_numerical(self, q: np.ndarray) -> np.ndarray:
        """
        Simplified geometric FK fallback when KDL is unavailable.

        Uses joint origin translations from the URDF (not the simplified
        spherical model from the first version).  Still approximate because
        it ignores the RPY offsets between joints.

        ⚠ This WILL produce wrong Jacobian directions (especially Y-axis)
        because the actual joint orientations are non-trivial.  If you see
        this warning, install PyKDL and kdl_parser_py.
        """
        # Link lengths from URDF joint origin xyz values
        # J2 origin: xyz="-0.28857 0 0"
        # J3 origin: xyz="0.15351 0.074002 0" → L≈0.170
        # J4 origin: xyz="0 0 0.05447"
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

    # ── Jacobian ──────────────────────────────────────────────────────────────

    def _jacobian_kdl(self, q: np.ndarray) -> np.ndarray | None:
        """
        Return the 3×N linear-velocity Jacobian via KDL.

        Uses jac[r, c] element-wise access (universally supported across all
        PyKDL versions).  Previous getColumn(c).vel.x() approach silently
        failed on some builds, causing fallback to the wrong numerical model.
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
            # Rows 0-2 = linear velocity; rows 3-5 = angular velocity (not used here)
            J = np.array([[jac[r, c] for c in range(n)] for r in range(3)])
        except Exception as e:
            self.get_logger().warn(f'Jacobian extraction failed ({e}) – falling back.')
            return None

        return J

    def _jacobian_numerical(self, q: np.ndarray, eps: float = 1e-4) -> np.ndarray:
        p0 = self._fk_numerical(q)
        J = np.zeros((3, self.N_JOINTS))
        for i in range(self.N_JOINTS):
            dq = q.copy()
            dq[i] += eps
            J[:, i] = (self._fk_numerical(dq) - p0) / eps
        return J

    def _get_jacobian(self, q: np.ndarray) -> np.ndarray | None:
        if self._jac_solver is not None:
            return self._jacobian_kdl(q)
        self.get_logger().warn(
            'KDL not available – numerical Jacobian (Y-axis may be wrong!)',
            throttle_duration_sec=5.0,
        )
        return self._jacobian_numerical(q)

    # ── Main control loop (33 Hz) ─────────────────────────────────────────────

    def _control_loop(self):
        if self._q is None:
            return

        # First valid joint state: initialise the integrator
        if self._q_cmd is None:
            self._q_cmd = self._q.copy()
            self._moving = False

        # ── Dead-zone ─────────────────────────────────────────────────────────
        deadzone = self.get_parameter('deadzone').get_parameter_value().double_value
        if float(np.linalg.norm(self._cmd_vel)) < deadzone:
            if self._moving:
                # Hold current command position
                self._publish_traj(self._q_cmd.tolist(), [0.0] * self.N_JOINTS, 0.1)
                self._moving = False
            return

        # Sync integrator to actual state at start of each motion burst.
        # This prevents a sudden jump if the arm was moved externally while paused.
        if not self._moving:
            self._q_cmd = self._q.copy()
            self._moving = True

        if not self._linear_mode:
            # Angular / orientation mode – future work, skip silently
            return

        # ── Parameters ────────────────────────────────────────────────────────
        max_cv = self.get_parameter('max_cartesian_vel').get_parameter_value().double_value
        max_jv = self.get_parameter('max_joint_vel').get_parameter_value().double_value
        lam = self.get_parameter('jacobian_damping').get_parameter_value().double_value
        horizon = self.get_parameter('trajectory_horizon_sec').get_parameter_value().double_value
        horizon = max(horizon, self._dt * 2.0)  # never shorter than 2 control ticks

        # Scale joystick [-1,1] → Cartesian velocity (m/s) in arm_base_link frame
        cart_vel = self._cmd_vel * max_cv

        # ── Jacobian (computed at integrator state, not sensor state) ─────────
        # Using _q_cmd keeps linearisation consistent with integration below.
        J = self._get_jacobian(self._q_cmd)
        if J is None:
            return

        # ── Damped least-squares IK ───────────────────────────────────────────
        # dq = J^T (J J^T + λ² I)^-1 v_cart
        try:
            JJt = J @ J.T  # 3×3
            dq = J.T @ np.linalg.solve(JJt + lam**2 * np.eye(3), cart_vel)  # rad/s
        except np.linalg.LinAlgError:
            self.get_logger().warn('DLS solve failed.')
            return

        # ── Uniform velocity scaling (preserves EE direction) ─────────────────
        peak = float(np.max(np.abs(dq)))
        vel_scale = 1.0
        if peak > max_jv:
            vel_scale = max_jv / peak
            dq *= vel_scale

        # ── Per-joint limit clamping (KEY BUG FIX) ────────────────────────────
        #
        # WRONG (old): global limit_scale = min over all joints
        #   → J2 near lower limit at home → scale ≈ 0.05 → ALL axes die
        #
        # RIGHT (new): clamp each joint independently
        #   → J2 at lower limit: zero J2's velocity ONLY; J0, J1, … continue
        #
        joints_clipped: list[str] = []
        for i in range(self.N_JOINTS):
            q_next = self._q_cmd[i] + dq[i] * self._dt
            if q_next > self._q_hi[i] and dq[i] > 0.0:
                # Approaching upper limit: reduce to exactly reach the boundary
                dq[i] = max(0.0, (self._q_hi[i] - self._q_cmd[i]) / self._dt)
                joints_clipped.append(f'J{i}↑')
            elif q_next < self._q_lo[i] and dq[i] < 0.0:
                # Approaching lower limit: reduce to exactly reach the boundary
                dq[i] = min(0.0, (self._q_lo[i] - self._q_cmd[i]) / self._dt)
                joints_clipped.append(f'J{i}↓')

        # ── Integrate command state ────────────────────────────────────────────
        self._q_cmd = np.clip(self._q_cmd + dq * self._dt, self._q_lo, self._q_hi)

        # ── Diagnostics (1 Hz) ────────────────────────────────────────────────
        self._diag_ctr += 1
        if self._diag_ctr % 33 == 0:
            ee = self._get_ee_pos(self._q_cmd)
            achieved_cart = J @ dq  # approximate – J is at q_cmd, not q_cmd+delta
            self.get_logger().info(
                f'cart_in={np.round(cart_vel, 3)} '
                f'dq={np.round(dq, 3)} '
                f'cart_out={np.round(achieved_cart, 3)} '
                f'vel_scale={vel_scale:.2f} '
                f'clipped={joints_clipped}\n'
                f'  q_cmd={np.round(self._q_cmd, 3)}\n'
                f'  q_meas={np.round(self._q, 3)}\n'
                f'  EE={np.round(ee, 4)} m'
            )

        # ── Publish ───────────────────────────────────────────────────────────
        if self._command_mode == 'velocity':
            self._publish_velocity(dq.tolist())
        else:
            self._publish_traj(self._q_cmd.tolist(), dq.tolist(), horizon)

    # ── Trajectory publisher ──────────────────────────────────────────────────

    def _publish_traj(self, positions: list[float], velocities: list[float], duration_sec: float):
        """
        Publish a 2-point trajectory to the JointTrajectoryController.

        Point 1 (at duration_sec):   target position + current velocity hint
        Point 2 (at duration_sec+0.15): same position, zero velocity (required
            by most JTC implementations to indicate the final resting state)

        BUG FIX: stop-point uses the SAME position as point 1 (clipped to
        limits), not position + velocity*extrapolation which could go
        out-of-bounds and cause JTC to reject the entire trajectory.
        """
        traj = JointTrajectory()
        traj.header.stamp.sec = 0  # execute immediately, preempt current
        traj.header.stamp.nanosec = 0
        traj.header.frame_id = self._command_frame
        traj.joint_names = self.JOINT_NAMES

        # Safety clip: never send out-of-limit positions
        pos_safe = [
            float(np.clip(positions[i], self._q_lo[i], self._q_hi[i]))
            for i in range(self.N_JOINTS)
        ]

        pt = JointTrajectoryPoint()
        pt.positions = pos_safe
        pt.velocities = [float(v) for v in velocities]
        pt.time_from_start = Duration(nanoseconds=int(duration_sec * 1e9)).to_msg()

        # Stop point: same clipped position, zero velocity
        pt_stop = JointTrajectoryPoint()
        pt_stop.positions = pos_safe
        pt_stop.velocities = [0.0] * self.N_JOINTS
        pt_stop.time_from_start = Duration(nanoseconds=int((duration_sec + 0.15) * 1e9)).to_msg()

        traj.points = [pt, pt_stop]
        self._traj_pub.publish(traj)

    def _publish_velocity(self, velocities: list[float]):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in velocities]
        self._vel_pub.publish(msg)

    # ── Services ──────────────────────────────────────────────────────────────

    def _srv_home(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        dur = self.get_parameter('home_duration_sec').get_parameter_value().double_value
        # Reset integrator to home so the next velocity command starts from home
        self._q_cmd = np.array(self.HOME_POSITION, dtype=float)
        self._moving = False
        self._publish_traj(self.HOME_POSITION, [0.0] * self.N_JOINTS, dur)
        res.success = True
        res.message = f'Moving to home {self.HOME_POSITION} over {dur:.1f}s'
        self.get_logger().info(res.message)
        return res

    def _srv_switch_vel(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        self._linear_mode = req.data
        res.success = True
        res.message = 'Velocity → LINEAR' if req.data else 'Velocity → ANGULAR'
        self.get_logger().info(res.message)
        return res

    def _srv_set_mode(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        self._velocity_mode = req.data
        res.success = True
        res.message = 'Mode → VELOCITY' if req.data else 'Mode → POSITION INCREMENT'
        self.get_logger().info(res.message)
        return res

    def _srv_beak(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
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
