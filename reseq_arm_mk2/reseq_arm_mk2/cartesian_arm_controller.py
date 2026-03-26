#!/usr/bin/env python3
"""
Cartesian arm controller for the RESE.Q MK2 arm.

It reads joint states, turns Cartesian velocity commands into joint motion
with a damped Jacobian solve, and publishes the result to the arm
controller. It also exposes a few small services for homing, beak control,
and mode switching.
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

    # Home is kept comfortably away from the joint limits.
    HOME_POSITION = [0.8, 0.0, 0.8, 0.0, 0.0, 0.0]
    JOINT_WEIGHTS = np.array([3.0, 2.5, 1.4, 1.0, 0.7, 0.7], dtype=float)

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
        self.declare_parameter('deadzone', 0.02)
        self.declare_parameter('jacobian_damping', 0.05)

        # Internal state.
        self._q: np.ndarray | None = None  # measured joint positions
        self._q_cmd: np.ndarray | None = None  # integrator state
        self._cmd_vel = np.zeros(3)  # latest joystick command
        self._linear_mode = True
        self._velocity_mode = True
        self._moving = False
        self._diag_ctr = 0

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
        self._beak_pub = self.create_publisher(Int32, 'reseq/module33/mk2_arm/beak/setpoint', 10)

        self.create_service(Trigger, '/cartesian_arm_controller/go_home', self._srv_home)
        self.create_service(SetBool, '/cartesian_arm_controller/switch_vel', self._srv_switch_vel)
        self.create_service(SetBool, '/cartesian_arm_controller/set_mode', self._srv_set_mode)
        self.create_service(SetBool, '/cartesian_arm_controller/close_beak', self._srv_beak)

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
            self._q_cmd = self._q.copy()
            self._moving = False

        # Ignore tiny inputs.
        deadzone = self.get_parameter('deadzone').get_parameter_value().double_value
        if float(np.linalg.norm(self._cmd_vel)) < deadzone:
            if self._command_mode == 'velocity':
                self._publish_velocity([0.0] * self.N_JOINTS)
            elif self._moving:
                # Keep the current target when the stick returns to center.
                self._publish_traj(self._q.tolist(), self._q_cmd.tolist(), 0.1)
                self._moving = False
            return

        # Solve from the live measured pose so the Jacobian tracks the actual arm.
        if not self._moving:
            self._moving = True
        solve_q = self._q

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

        # Use the command-side state so the linearization matches the update step.
        J = self._get_jacobian(solve_q)
        if J is None:
            return

        # Damped least-squares IK on the tool translational task.
        try:
            active_dofs = J.shape[1]
            Jlin = J[:3, :active_dofs]
            joint_weights = self.JOINT_WEIGHTS[:active_dofs]
            inv_joint_weights = np.diag(1.0 / joint_weights)
            JJt = Jlin @ inv_joint_weights @ Jlin.T
            dq_active = (
                inv_joint_weights @ Jlin.T @ np.linalg.solve(JJt + lam**2 * np.eye(3), cart_vel)
            )

            dq = np.zeros(self.N_JOINTS)
            dq[:active_dofs] = dq_active
        except np.linalg.LinAlgError:
            self.get_logger().warn('DLS solve failed.')
            return

        # Scale the whole command if any joint exceeds the velocity limit.
        peak = float(np.max(np.abs(dq[:active_dofs])))
        vel_scale = 1.0
        if peak > max_jv:
            vel_scale = max_jv / peak
            dq[:active_dofs] *= vel_scale

        # Clamp each joint independently.
        joints_clipped: list[str] = []
        for i in range(active_dofs):
            q_next = solve_q[i] + dq[i] * self._dt
            if q_next > self._q_hi[i] and dq[i] > 0.0:
                # Approaching upper limit: reduce to exactly reach the boundary
                dq[i] = max(0.0, (self._q_hi[i] - solve_q[i]) / self._dt)
                joints_clipped.append(f'J{i}↑')
            elif q_next < self._q_lo[i] and dq[i] < 0.0:
                # Approaching lower limit: reduce to exactly reach the boundary
                dq[i] = min(0.0, (self._q_lo[i] - solve_q[i]) / self._dt)
                joints_clipped.append(f'J{i}↓')

        # Advance the target by the trajectory horizon so the published
        # waypoint matches the intended servo speed.
        self._q_cmd = np.clip(solve_q + dq * horizon, self._q_lo, self._q_hi)

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
                f'clipped={joints_clipped}\n'
                f'  q_cmd={np.round(self._q_cmd, 3)}\n'
                f'  q_meas={np.round(self._q, 3)}\n'
                f'  EE={np.round(ee, 4)} m'
            )

        # Publish the command.
        if self._command_mode == 'velocity':
            self._publish_velocity(dq.tolist())
        else:
            self._publish_traj(self._q.tolist(), self._q_cmd.tolist(), horizon)

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

    # Services.

    def _srv_home(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        dur = self.get_parameter('home_duration_sec').get_parameter_value().double_value
        # Reset the integrator so the next move starts from home.
        self._q_cmd = np.array(self.HOME_POSITION, dtype=float)
        self._moving = False
        if self._command_mode == 'velocity':
            self._publish_velocity([0.0] * self.N_JOINTS)
            res.success = True
            res.message = 'Velocity mode: command state reset and motion stopped'
        else:
            start_positions = self._q.tolist() if self._q is not None else self.HOME_POSITION
            self._publish_traj(start_positions, self.HOME_POSITION, dur)
            res.success = True
            res.message = f'Moving to home {self.HOME_POSITION} over {dur:.1f}s'
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

    def _srv_beak(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        self._beak_pub.publish(Int32(data=int(req.data)))
        res.success = True
        res.message = f'Beak -> {"CLOSE" if req.data else "OPEN"}'
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
