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


def _idle_hold_command(
    current_q: np.ndarray,
    hold_target: np.ndarray,
    q_lo: np.ndarray,
    q_hi: np.ndarray,
    dt: float,
    gain: float,
    max_joint_vel: float,
    tolerance: float,
    hold_armed: bool,
) -> tuple[np.ndarray, np.ndarray]:
    """Return the idle hold target and velocity command.

    Before the operator has moved the arm, a captured startup pose is only an
    observation. On real hardware the first joint state can briefly contain the
    hardware interface's zero-initialized buffers, so an unarmed hold must never
    drive back toward that value.
    """
    if not hold_armed:
        passive_target = np.array(current_q, dtype=float, copy=True)
        return passive_target, np.zeros_like(current_q)

    target = np.array(hold_target, dtype=float, copy=True)
    dq = _compute_joint_hold_velocity(
        current_q=current_q,
        target_q=target,
        q_lo=q_lo,
        q_hi=q_hi,
        dt=dt,
        gain=gain,
        max_joint_vel=max_joint_vel,
        tolerance=tolerance,
    )
    return target, dq


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


def _rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return a base-to-tool rotation matrix from fixed-axis RPY angles."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _rotation_error_vector(current_rotation: np.ndarray, desired_rotation: np.ndarray) -> np.ndarray:
    """Return the base-frame angular correction from current to desired orientation."""
    r_err = desired_rotation @ current_rotation.T
    cos_theta = float(np.clip((np.trace(r_err) - 1.0) * 0.5, -1.0, 1.0))
    theta = float(np.arccos(cos_theta))
    skew_vec = np.array(
        [
            r_err[2, 1] - r_err[1, 2],
            r_err[0, 2] - r_err[2, 0],
            r_err[1, 0] - r_err[0, 1],
        ]
    )
    if theta < 1e-6:
        return 0.5 * skew_vec

    sin_theta = float(np.sin(theta))
    if abs(sin_theta) < 1e-6:
        return np.zeros(3)
    return (theta / (2.0 * sin_theta)) * skew_vec


def _forward_axis_error_vector(
    current_rotation: np.ndarray,
    desired_rotation: np.ndarray,
) -> np.ndarray:
    """Return angular correction that points the tool forward axis at robot-forward."""
    current_forward = current_rotation[:, 0]
    desired_forward = desired_rotation[:, 0]
    current_norm = np.linalg.norm(current_forward)
    desired_norm = np.linalg.norm(desired_forward)
    if current_norm < 1e-9 or desired_norm < 1e-9:
        return np.zeros(3)
    current_forward = current_forward / current_norm
    desired_forward = desired_forward / desired_norm
    return np.cross(current_forward, desired_forward)


def _forward_axis_alignment_error(
    current_rotation: np.ndarray,
    desired_rotation: np.ndarray,
) -> float:
    """Return the angular error between the current and desired forward axes."""
    current_forward = current_rotation[:, 0]
    desired_forward = desired_rotation[:, 0]
    current_norm = np.linalg.norm(current_forward)
    desired_norm = np.linalg.norm(desired_forward)
    if current_norm < 1e-9 or desired_norm < 1e-9:
        return 0.0
    current_forward = current_forward / current_norm
    desired_forward = desired_forward / desired_norm
    cos_error = float(np.clip(np.dot(current_forward, desired_forward), -1.0, 1.0))
    return float(np.arccos(cos_error))


def _forward_progress_is_acceptable(
    current_error: float,
    next_error: float,
    tolerance: float = 0.02,
    epsilon: float = 1e-4,
) -> bool:
    """Return whether a predicted command preserves or recovers forward-look."""
    if next_error <= tolerance:
        return True
    if next_error > current_error + epsilon:
        return False
    if current_error > tolerance and next_error >= current_error - epsilon:
        return False
    return True


def _task_velocity_direction_is_acceptable(
    desired_vel: np.ndarray,
    achieved_vel: np.ndarray,
    deadzone: float,
) -> bool:
    """Return whether the dominant commanded task axis still moves as requested."""
    desired = np.asarray(desired_vel, dtype=float).reshape(-1)
    achieved = np.asarray(achieved_vel, dtype=float).reshape(-1)
    if desired.size == 0 or achieved.size != desired.size:
        return True

    axis = int(np.argmax(np.abs(desired)))
    desired_axis = float(desired[axis])
    if abs(desired_axis) <= deadzone:
        return True
    achieved_axis = float(achieved[axis])
    return achieved_axis * desired_axis > 1e-6


def _dominant_rotation_input(cmd_vel: np.ndarray, deadzone: float) -> np.ndarray:
    """Keep full-stick rotation commands on one tool axis."""
    filtered = np.array(cmd_vel, dtype=float, copy=True)
    if deadzone <= 0.0:
        return filtered

    magnitudes = np.abs(filtered)
    dominant = int(np.argmax(magnitudes))
    dominant_mag = float(magnitudes[dominant])
    if dominant_mag <= deadzone:
        return np.zeros_like(filtered)

    cross_axis_limit = max(deadzone, 0.35 * dominant_mag)
    filtered[magnitudes < cross_axis_limit] = 0.0
    return filtered


def _rotation_mode_angular_velocity(
    cmd_vel: np.ndarray,
    max_angular_vel: float,
    deadzone: float = 0.0,
) -> np.ndarray:
    """Map scaler arm input to tool-frame roll, tilt, and pan angular velocity."""
    filtered_cmd = _dominant_rotation_input(cmd_vel, deadzone)
    return max_angular_vel * np.array(
        [
            filtered_cmd[0],  # forward/back stick rolls around tool X
            filtered_cmd[2],  # Z control tilts around tool Y
            filtered_cmd[1],  # left/right stick pans around tool Z
        ],
        dtype=float,
    )


def _clip_vector(vec: np.ndarray, limit: float) -> np.ndarray:
    return np.clip(vec, -abs(limit), abs(limit))


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


def _weighted_dls_pseudoinverse(
    task_jacobian: np.ndarray,
    joint_weights: np.ndarray,
    damping: float,
) -> np.ndarray:
    """Return the weighted DLS pseudo-inverse for a task Jacobian."""
    inv_joint_weights = np.diag(1.0 / joint_weights)
    jj_t = task_jacobian @ inv_joint_weights @ task_jacobian.T
    return (
        inv_joint_weights
        @ task_jacobian.T
        @ np.linalg.inv(jj_t + damping**2 * np.eye(task_jacobian.shape[0]))
    )


def _append_secondary_task(
    secondary_jacobian: np.ndarray | None,
    secondary_vel: np.ndarray | None,
    task_jacobian: np.ndarray,
    task_vel: np.ndarray,
    weight: float,
) -> tuple[np.ndarray | None, np.ndarray | None]:
    """Append a weighted secondary task, preserving an empty task as None."""
    if weight <= 0.0 or task_jacobian.size == 0 or task_vel.size == 0:
        return secondary_jacobian, secondary_vel

    weighted_jacobian = weight * task_jacobian
    weighted_vel = weight * task_vel
    if secondary_jacobian is None or secondary_vel is None:
        return weighted_jacobian, weighted_vel
    return (
        np.vstack([secondary_jacobian, weighted_jacobian]),
        np.concatenate([secondary_vel, weighted_vel]),
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


def _solve_prioritized_task_velocity(
    current_q: np.ndarray,
    primary_jacobian: np.ndarray,
    primary_vel: np.ndarray,
    secondary_jacobian: np.ndarray | None,
    secondary_vel: np.ndarray | None,
    q_lo: np.ndarray,
    q_hi: np.ndarray,
    dt: float,
    max_joint_vel: float,
    damping: float,
    joint_weights: np.ndarray,
    secondary_weight: float,
) -> tuple[np.ndarray, list[str], float]:
    """Solve the primary task and optionally add a nullspace secondary task."""
    dq, joints_clipped, vel_scale = _solve_task_velocity_with_limit_redistribution(
        current_q=current_q,
        task_jacobian=primary_jacobian,
        cart_vel=primary_vel,
        q_lo=q_lo,
        q_hi=q_hi,
        dt=dt,
        max_joint_vel=max_joint_vel,
        damping=damping,
        joint_weights=joint_weights,
    )

    if (
        secondary_jacobian is None
        or secondary_vel is None
        or secondary_weight <= 0.0
        or secondary_jacobian.size == 0
    ):
        return dq, joints_clipped, vel_scale

    pinv_primary = _weighted_dls_pseudoinverse(primary_jacobian, joint_weights, damping)
    nullspace = np.eye(primary_jacobian.shape[1]) - pinv_primary @ primary_jacobian
    residual = secondary_vel - secondary_jacobian @ dq
    projected_secondary = secondary_jacobian @ nullspace
    if np.linalg.norm(projected_secondary) < 1e-9 or np.linalg.norm(residual) < 1e-9:
        return dq, joints_clipped, vel_scale

    dq_secondary = nullspace @ _solve_weighted_dls_task_velocity(
        task_jacobian=projected_secondary,
        cart_vel=secondary_weight * residual,
        joint_weights=joint_weights,
        damping=damping,
    )
    remaining_budget = np.maximum(max_joint_vel - np.abs(dq), 0.0)
    secondary_scale = 1.0
    for idx, value in enumerate(dq_secondary):
        magnitude = abs(float(value))
        if magnitude > 1e-12:
            secondary_scale = min(secondary_scale, remaining_budget[idx] / magnitude)
    dq_secondary *= float(np.clip(secondary_scale, 0.0, 1.0))

    dq_candidate = dq + dq_secondary
    dq_candidate = _clamp_joint_velocity_to_limits(
        current_q, dq_candidate, q_lo, q_hi, dt
    )
    return dq_candidate, joints_clipped, vel_scale


def _remove_task_space_velocity(
    dq: np.ndarray,
    task_jacobian: np.ndarray,
    joint_weights: np.ndarray,
    damping: float,
) -> np.ndarray:
    """Project a command back onto the zero-velocity manifold for a task."""
    residual = task_jacobian @ dq
    if np.linalg.norm(residual) < 1e-9:
        return dq
    correction = _solve_weighted_dls_task_velocity(
        task_jacobian=task_jacobian,
        cart_vel=residual,
        joint_weights=joint_weights,
        damping=damping,
    )
    return dq - correction


def _limit_task_space_velocity(
    dq: np.ndarray,
    task_jacobian: np.ndarray,
    max_task_speed: float,
    joint_weights: np.ndarray,
    damping: float,
) -> np.ndarray:
    """Limit uncommanded task-space drift while preserving as much dq as possible."""
    task_vel = task_jacobian @ dq
    speed = float(np.linalg.norm(task_vel))
    if speed <= max_task_speed or speed < 1e-9:
        return dq

    allowed_vel = task_vel * (max_task_speed / speed)
    correction = _solve_weighted_dls_task_velocity(
        task_jacobian=task_jacobian,
        cart_vel=task_vel - allowed_vel,
        joint_weights=joint_weights,
        damping=damping,
    )
    return dq - correction


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


def _linear_startup_escape_velocity(
    current_q: np.ndarray,
    q_lo: np.ndarray,
    max_joint_vel: float,
    cart_vel_cmd: np.ndarray,
    deadzone: float,
    margin: float = 0.25,
    wrist_margin: float = 0.08,
) -> np.ndarray:
    """Bias folded startup Z motion toward an arm shape that can actually rise."""
    escape = np.zeros_like(current_q)
    if len(current_q) < 5:
        return escape

    if not _is_dominant_positive_z_command(cart_vel_cmd, deadzone):
        return escape

    elbow_distance = float(current_q[2] - q_lo[2])
    if elbow_distance >= margin:
        return escape

    strength = float(np.clip((margin - elbow_distance) / margin, 0.0, 1.0))
    escape[0] = -0.20 * max_joint_vel * strength
    escape[2] = 0.95 * max_joint_vel * strength

    wrist_room = float(current_q[4] - q_lo[4])
    if wrist_room > 0.0:
        wrist_scale = float(np.clip(wrist_room / wrist_margin, 0.0, 1.0))
        escape[4] = -0.15 * max_joint_vel * strength * wrist_scale
    return escape


def _is_lower_elbow_positive_z_recovery_active(
    current_q: np.ndarray,
    q_lo: np.ndarray,
    cart_vel_cmd: np.ndarray,
    deadzone: float,
    margin: float = 0.03,
) -> bool:
    return (
        len(current_q) > 4
        and _is_dominant_positive_z_command(cart_vel_cmd, deadzone)
        and float(current_q[2]) <= float(q_lo[2]) + margin
    )


def _lower_elbow_positive_z_escape_velocity(
    current_q: np.ndarray,
    q_lo: np.ndarray,
    q_hi: np.ndarray,
    max_joint_vel: float,
    shoulder_reserve: float = 0.18,
    wrist_reserve: float = 0.18,
) -> np.ndarray:
    """Lift from the folded lower-elbow stop without depending on elbow motion."""
    escape = np.zeros_like(current_q)
    if len(current_q) < 5:
        return escape

    if current_q[0] > q_lo[0] + shoulder_reserve:
        escape[0] = -0.35 * max_joint_vel
    if current_q[2] < q_hi[2] - 1e-4:
        escape[2] = max_joint_vel
    if current_q[4] > q_lo[4] + wrist_reserve:
        escape[4] = -0.35 * max_joint_vel
    return escape


def _lower_elbow_corner_unstick_velocity(
    current_q: np.ndarray,
    q_lo: np.ndarray,
    q_hi: np.ndarray,
    max_joint_vel: float,
    shoulder_reserve: float = 0.18,
    wrist_reserve: float = 0.18,
) -> np.ndarray:
    """Open a tucked lower-elbow corner before trying to optimize vertical FK."""
    escape = np.zeros_like(current_q)
    if len(current_q) < 5:
        return escape

    if current_q[0] <= q_lo[0] + shoulder_reserve:
        escape[0] = 0.45 * max_joint_vel
    if current_q[2] < q_hi[2] - 1e-4:
        escape[2] = 0.75 * max_joint_vel
    if current_q[4] <= q_lo[4] + wrist_reserve:
        escape[4] = 0.45 * max_joint_vel
    return escape


def _lower_elbow_command_spends_stop_reserve(
    current_q: np.ndarray,
    dq: np.ndarray,
    q_lo: np.ndarray,
    shoulder_reserve: float = 0.18,
    wrist_reserve: float = 0.18,
) -> bool:
    if len(current_q) < 5 or len(dq) < 5:
        return False

    shoulder_spends = (
        current_q[0] <= q_lo[0] + shoulder_reserve
        and dq[0] < -1e-6
    )
    wrist_spends = (
        current_q[4] <= q_lo[4] + wrist_reserve
        and dq[4] < -1e-6
    )
    return bool(shoulder_spends or wrist_spends)


def _is_lower_elbow_hard_corner(
    current_q: np.ndarray,
    q_lo: np.ndarray,
    shoulder_margin: float = 0.04,
    wrist_margin: float = 0.08,
) -> bool:
    if len(current_q) < 5:
        return False
    return bool(
        current_q[0] <= q_lo[0] + shoulder_margin
        or current_q[4] <= q_lo[4] + wrist_margin
    )


def _dominant_z_lift_velocity(
    cart_vel: np.ndarray,
    current_q: np.ndarray,
    q_lo: np.ndarray,
    max_cartesian_vel: float,
    cart_vel_cmd: np.ndarray,
    deadzone: float,
    elbow_margin: float = 0.35,
    shoulder_unfolded: float = 0.75,
) -> np.ndarray:
    """Boost folded positive-Z lift without injecting forward motion."""
    lifted = np.array(cart_vel, dtype=float, copy=True)
    if len(current_q) < 3 or not _is_dominant_positive_z_command(cart_vel_cmd, deadzone):
        return lifted

    elbow_distance = float(current_q[2] - q_lo[2])
    elbow_fold = float(np.clip((elbow_margin - elbow_distance) / elbow_margin, 0.0, 1.0))
    shoulder_fold = float(np.clip((shoulder_unfolded - current_q[0]) / shoulder_unfolded, 0.0, 1.0))
    strength = max(elbow_fold, shoulder_fold)
    if strength <= 0.0:
        return lifted

    min_z_speed = 0.75 * max_cartesian_vel * strength
    if lifted[2] > 0.0:
        lifted[2] = min(max_cartesian_vel, max(lifted[2], min_z_speed))
    return lifted


def _solve_directional_z_velocity(
    current_q: np.ndarray,
    z_jacobian: np.ndarray,
    desired_z_vel: float,
    q_lo: np.ndarray,
    q_hi: np.ndarray,
    dt: float,
    max_joint_vel: float,
    damping: float,
    joint_weights: np.ndarray,
) -> tuple[np.ndarray, list[str], float]:
    """Solve Z using only joints whose instantaneous motion helps the requested direction."""
    z_row = np.asarray(z_jacobian, dtype=float).reshape(-1)
    helpful = np.abs(z_row) > 1e-9
    if not np.any(helpful):
        return np.zeros_like(current_q), ['Z↕no_helpful_joint'], 0.0

    helpful_indices = np.flatnonzero(helpful)
    dq_helpful, clipped, scale = _solve_task_velocity_with_limit_redistribution(
        current_q=current_q[helpful_indices],
        task_jacobian=z_jacobian[:, helpful_indices],
        cart_vel=np.array([desired_z_vel]),
        q_lo=q_lo[helpful_indices],
        q_hi=q_hi[helpful_indices],
        dt=dt,
        max_joint_vel=max_joint_vel,
        damping=damping,
        joint_weights=joint_weights[helpful_indices],
    )
    dq = np.zeros_like(current_q)
    dq[helpful_indices] = dq_helpful
    mapped_clipped = []
    for label in clipped:
        if label.startswith('J') and len(label) >= 3:
            try:
                local_idx = int(label[1:-1])
            except ValueError:
                mapped_clipped.append(label)
            else:
                mapped_clipped.append(f'J{helpful_indices[local_idx]}{label[-1]}')
        else:
            mapped_clipped.append(label)
    return dq, mapped_clipped, scale


def _minimum_positive_z_progress(desired_z_vel: float) -> float:
    return 0.25 * abs(float(desired_z_vel))


def _linear_command_target_dt(
    command_mode: str,
    horizon: float,
    control_dt: float,
    cart_vel_cmd: np.ndarray,
    deadzone: float,
) -> float:
    if (
        command_mode == 'trajectory'
        and _is_dominant_positive_z_command(cart_vel_cmd, deadzone)
    ):
        return control_dt
    return horizon


def _linear_lateral_forward_velocity(
    cart_vel: np.ndarray,
    cart_vel_cmd: np.ndarray,
    max_cartesian_vel: float,
    deadzone: float,
    front_branch_gain: float,
) -> np.ndarray:
    """Keep manual lateral commands literal; no artificial X branch bias."""
    del cart_vel_cmd, max_cartesian_vel, deadzone, front_branch_gain
    return np.array(cart_vel, dtype=float, copy=True)


def _is_lateral_forward_bias_active(cart_vel_cmd: np.ndarray, deadzone: float) -> bool:
    del cart_vel_cmd, deadzone
    return False


def _is_dominant_positive_z_command(cart_vel_cmd: np.ndarray, deadzone: float) -> bool:
    z_cmd = float(cart_vel_cmd[2])
    lateral_cmd = float(max(abs(cart_vel_cmd[0]), abs(cart_vel_cmd[1])))
    return z_cmd > deadzone and z_cmd >= lateral_cmd


def _is_dominant_z_command(cart_vel_cmd: np.ndarray, deadzone: float) -> bool:
    z_cmd = abs(float(cart_vel_cmd[2]))
    lateral_cmd = float(max(abs(cart_vel_cmd[0]), abs(cart_vel_cmd[1])))
    return z_cmd > deadzone and z_cmd >= lateral_cmd


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
    velocity_topic         str    '/mk2_arm_vel'
    state_topic            str    '/arm_joint_states'
    trajectory_topic       str    '/mk2_arm_controller/joint_trajectory'
    control_rate           float  33.0   Hz
    max_cartesian_vel      float  0.3    m/s  (joystick [-1,1] scaled by this)
    max_joint_vel          float  1.0    rad/s per joint
    max_angular_vel        float  0.8    rad/s for camera pan/tilt/roll commands
    home_duration_sec      float  3.0    s
    trajectory_horizon_sec float  0.10   s
    command_mode           str    'trajectory' or 'velocity'
    deadzone               float  0.02
    jacobian_damping       float  0.05   damped-LS regularisation λ
    joint_weights          float[]  joint weighting for the IK solve
    robot_forward_rpy      float[] fixed tool orientation for "camera forward"
    orientation_hold_gain  float  proportional gain for forward orientation hold
    orientation_task_weight float nullspace weight for linear-mode orientation hold
    linear_posture_target  float[] nullspace posture target for linear mode
    linear_posture_weight  float  weight for linear-mode posture task
    linear_posture_gain    float  proportional gain for linear posture task
    position_hold_gain     float  proportional gain for rotation-mode position hold
    secondary_task_weight  float  weight for nullspace angular task
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
        self.declare_parameter('velocity_topic', '/mk2_arm_vel')
        self.declare_parameter('state_topic', '/arm_joint_states')
        self.declare_parameter('trajectory_topic', '/mk2_arm_controller/joint_trajectory')
        self.declare_parameter('control_rate', 33.0)
        self.declare_parameter('max_cartesian_vel', 0.6)
        self.declare_parameter('max_angular_vel', 0.8)
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
        self.declare_parameter('robot_forward_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('orientation_hold_gain', 2.0)
        self.declare_parameter('orientation_task_weight', 1.4)
        self.declare_parameter('linear_posture_target', self.HOME_POSITION)
        self.declare_parameter('linear_posture_weight', 0.2)
        self.declare_parameter('linear_posture_gain', 0.25)
        self.declare_parameter('position_hold_gain', 2.0)
        self.declare_parameter('secondary_task_weight', 1.0)

        # Internal state.
        self._q: np.ndarray | None = None  # measured joint positions
        self._q_continuous: np.ndarray | None = None  # unwraps periodic joints across ±pi
        self._q_cmd: np.ndarray | None = None  # integrator state
        self._cmd_vel = np.zeros(3)  # latest joystick command
        self._linear_mode = True
        self._velocity_mode = True
        self._moving = False
        self._ee_z_ref: float | None = None
        self._ee_pos_ref: np.ndarray | None = None
        self._front_x_ref: float | None = None
        self._front_branch_gain = (
            self.get_parameter('front_branch_gain').get_parameter_value().double_value
        )
        linear_posture_target = list(self.get_parameter('linear_posture_target').value)
        if len(linear_posture_target) != self.N_JOINTS:
            self.get_logger().warn(
                'linear_posture_target must have 6 values; using HOME_POSITION.'
            )
            linear_posture_target = self.HOME_POSITION
        self._linear_posture_target = np.array(linear_posture_target, dtype=float)
        if not np.all(np.isfinite(self._linear_posture_target)):
            self.get_logger().warn(
                'linear_posture_target contains non-finite values; using HOME_POSITION.'
            )
            self._linear_posture_target = np.array(self.HOME_POSITION, dtype=float)
        forward_rpy = list(self.get_parameter('robot_forward_rpy').value)
        if len(forward_rpy) != 3:
            self.get_logger().warn('robot_forward_rpy must have 3 values; using [0, 0, 0].')
            forward_rpy = [0.0, 0.0, 0.0]
        self._forward_rotation = _rotation_matrix_from_rpy(
            float(forward_rpy[0]),
            float(forward_rpy[1]),
            float(forward_rpy[2]),
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
        self._idle_hold_armed = self._startup_hold_target is not None

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
        velocity_topic = self.get_parameter('velocity_topic').get_parameter_value().string_value
        state_topic = self.get_parameter('state_topic').get_parameter_value().string_value

        # ROS interfaces.
        self.create_subscription(Vector3, velocity_topic, self._cb_vel, 10)
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
            elif self._startup_hold_target is None and not self._idle_hold_armed:
                self._startup_measured_pose = self._q.copy()
                self._startup_home = self._q.copy()
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

    def _rotation_kdl_to_matrix(self, rotation) -> np.ndarray:
        return np.array([[rotation[r, c] for c in range(3)] for r in range(3)], dtype=float)

    def _get_ee_rotation(self, q: np.ndarray) -> np.ndarray | None:
        if self._fk_solver is None:
            return None
        frame = self._fk_frame_kdl(q)
        if frame is None:
            return None
        return self._rotation_kdl_to_matrix(frame.M)

    def _next_q_from_dq(
        self,
        current_q: np.ndarray,
        dq: np.ndarray,
        dt: float,
        active_dofs: int,
    ) -> np.ndarray:
        next_q = np.array(current_q, dtype=float, copy=True)
        next_q[:active_dofs] = np.clip(
            current_q[:active_dofs] + dq[:active_dofs] * dt,
            self._q_lo[:active_dofs],
            self._q_hi[:active_dofs],
        )
        return next_q

    def _forward_error_at_q(self, q: np.ndarray) -> float | None:
        rotation = self._get_ee_rotation(q)
        if rotation is None:
            return None
        return _forward_axis_alignment_error(rotation, self._forward_rotation)

    def _enforce_linear_forward_progress(
        self,
        current_q: np.ndarray,
        dq: np.ndarray,
        angular_jacobian: np.ndarray,
        angular_vel: np.ndarray,
        linear_task_jacobian: np.ndarray,
        linear_task_vel: np.ndarray,
        joint_weights: np.ndarray,
        max_joint_vel: float,
        damping: float,
        deadzone: float,
        active_dofs: int,
    ) -> tuple[np.ndarray, list[str], float, float | None, float | None]:
        current_error = self._forward_error_at_q(current_q)
        if current_error is None:
            return dq, [], 1.0, None, None

        next_error = self._forward_error_at_q(
            self._next_q_from_dq(current_q, dq, self._dt, active_dofs)
        )
        achieved_task_vel = linear_task_jacobian @ dq[:active_dofs]
        if (
            next_error is None
            or (
                _forward_progress_is_acceptable(current_error, next_error)
                and _task_velocity_direction_is_acceptable(
                    linear_task_vel, achieved_task_vel, deadzone
                )
            )
        ):
            return dq, [], 1.0, current_error, next_error

        best_dq = np.zeros_like(dq)
        best_error = current_error
        best_scale = 0.0
        best_clipped: list[str] = []
        best_preserves_direction = False
        for translation_scale in (1.0, 0.75, 0.5, 0.25, 0.1, 0.0):
            secondary_jacobian = None
            secondary_vel = None
            if translation_scale > 0.0:
                secondary_jacobian, secondary_vel = _append_secondary_task(
                    secondary_jacobian=secondary_jacobian,
                    secondary_vel=secondary_vel,
                    task_jacobian=linear_task_jacobian,
                    task_vel=translation_scale * linear_task_vel,
                    weight=1.0,
                )

            try:
                candidate_dq, candidate_clipped, _ = _solve_prioritized_task_velocity(
                    current_q=current_q,
                    primary_jacobian=angular_jacobian,
                    primary_vel=angular_vel,
                    secondary_jacobian=secondary_jacobian,
                    secondary_vel=secondary_vel,
                    q_lo=self._q_lo[:active_dofs],
                    q_hi=self._q_hi[:active_dofs],
                    dt=self._dt,
                    max_joint_vel=max_joint_vel,
                    damping=damping,
                    joint_weights=joint_weights,
                    secondary_weight=1.0,
                )
            except np.linalg.LinAlgError:
                continue

            candidate_error = self._forward_error_at_q(
                self._next_q_from_dq(current_q, candidate_dq, self._dt, active_dofs)
            )
            if candidate_error is None:
                continue
            candidate_task_vel = linear_task_jacobian @ candidate_dq[:active_dofs]
            direction_ok = _task_velocity_direction_is_acceptable(
                linear_task_vel, candidate_task_vel, deadzone
            )
            if (
                direction_ok
                and (
                    not best_preserves_direction
                    or candidate_error < best_error
                )
            ):
                best_dq = candidate_dq
                best_error = candidate_error
                best_scale = translation_scale
                best_clipped = candidate_clipped
                best_preserves_direction = True
            elif not best_preserves_direction and candidate_error < best_error:
                best_dq = candidate_dq
                best_error = candidate_error
                best_scale = translation_scale
                best_clipped = candidate_clipped
            if (
                direction_ok
                and _forward_progress_is_acceptable(current_error, candidate_error)
            ):
                return (
                    candidate_dq,
                    [*candidate_clipped, f'forward_guard:{translation_scale:.2f}'],
                    translation_scale,
                    current_error,
                    candidate_error,
                )

        if best_preserves_direction and best_error <= current_error + 1e-3:
            return (
                best_dq,
                [*best_clipped, f'forward_guard:{best_scale:.2f}'],
                best_scale,
                current_error,
                best_error,
            )
        # No scale produced direction-preserving motion. If the orientation error
        # is already small (below 2× tolerance), allow movement anyway — the arm
        # is close enough to correct orientation that blocking wastes all motion.
        if best_scale > 0.0 and current_error <= 2.0 * 0.02:
            return (
                best_dq,
                [*best_clipped, f'forward_guard:{best_scale:.2f}'],
                best_scale,
                current_error,
                best_error,
            )
        return np.zeros_like(dq), ['forward_hold'], 0.0, current_error, current_error

    def _enforce_positive_z_fk_progress(
        self,
        current_q: np.ndarray,
        dq: np.ndarray,
        cart_vel_cmd: np.ndarray,
        deadzone: float,
        max_joint_vel: float,
        active_dofs: int,
    ) -> tuple[np.ndarray, list[str]]:
        if not _is_dominant_positive_z_command(cart_vel_cmd, deadzone):
            return dq, []

        current_z = float(self._get_ee_pos(current_q)[2])
        pin_lower_elbow = _is_lower_elbow_positive_z_recovery_active(
            current_q=current_q[:active_dofs],
            q_lo=self._q_lo[:active_dofs],
            cart_vel_cmd=cart_vel_cmd,
            deadzone=deadzone,
        )

        def next_q_for(candidate_dq: np.ndarray) -> np.ndarray:
            candidate_next_q = self._next_q_from_dq(
                current_q,
                candidate_dq,
                self._dt,
                active_dofs,
            )
            if pin_lower_elbow:
                candidate_next_q[2] = current_q[2]
            return candidate_next_q

        def z_delta_for(candidate_dq: np.ndarray) -> float:
            return float(self._get_ee_pos(next_q_for(candidate_dq))[2]) - current_z

        current_delta = z_delta_for(dq)
        lower_elbow_hard_corner = pin_lower_elbow and _is_lower_elbow_hard_corner(
            current_q=current_q[:active_dofs],
            q_lo=self._q_lo[:active_dofs],
        )
        spends_stop_reserve = pin_lower_elbow and _lower_elbow_command_spends_stop_reserve(
            current_q=current_q[:active_dofs],
            dq=dq[:active_dofs],
            q_lo=self._q_lo[:active_dofs],
        )
        if lower_elbow_hard_corner or spends_stop_reserve:
            corner_unstick = _lower_elbow_corner_unstick_velocity(
                current_q=current_q[:active_dofs],
                q_lo=self._q_lo[:active_dofs],
                q_hi=self._q_hi[:active_dofs],
                max_joint_vel=max_joint_vel,
            )
            corner_unstick = _clamp_joint_velocity_to_limits(
                current_q[:active_dofs],
                corner_unstick,
                self._q_lo[:active_dofs],
                self._q_hi[:active_dofs],
                self._dt,
            )
            if np.linalg.norm(corner_unstick) > 1e-9:
                return corner_unstick, ['Z↑corner_unstick']
        if current_delta > 2e-4:
            return dq, []

        candidates: list[tuple[str, np.ndarray]] = []
        elbow_unstick = np.zeros(active_dofs)
        if active_dofs > 2 and current_q[2] < self._q_hi[2] - 1e-4:
            elbow_unstick[2] = max_joint_vel

        if pin_lower_elbow:
            lower_escape = _lower_elbow_positive_z_escape_velocity(
                current_q=current_q[:active_dofs],
                q_lo=self._q_lo[:active_dofs],
                q_hi=self._q_hi[:active_dofs],
                max_joint_vel=max_joint_vel,
            )
            candidates.append(('Z↑lower_escape', lower_escape))

            corner_unstick = _lower_elbow_corner_unstick_velocity(
                current_q=current_q[:active_dofs],
                q_lo=self._q_lo[:active_dofs],
                q_hi=self._q_hi[:active_dofs],
                max_joint_vel=max_joint_vel,
            )
            if np.linalg.norm(corner_unstick) > 1e-9:
                candidates.append(('Z↑corner_unstick', corner_unstick))

            q4_lift = np.zeros(active_dofs)
            if active_dofs > 4 and current_q[4] > self._q_lo[4] + 0.18:
                q4_lift[4] = -max_joint_vel
                candidates.append(('Z↑wrist_lift', q4_lift))

            q0_lift = np.zeros(active_dofs)
            if current_q[0] > self._q_lo[0] + 0.18:
                q0_lift[0] = -max_joint_vel
                candidates.append(('Z↑shoulder_lift', q0_lift))

            combined_lift = q0_lift + q4_lift
            if np.linalg.norm(combined_lift) > 1e-9:
                candidates.append(('Z↑shoulder_wrist_lift', combined_lift))
            if np.linalg.norm(elbow_unstick) > 1e-9:
                candidates.append(('Z↑elbow_unstick', elbow_unstick))

        startup_escape = _linear_startup_escape_velocity(
            current_q=current_q[:active_dofs],
            q_lo=self._q_lo[:active_dofs],
            max_joint_vel=max_joint_vel,
            cart_vel_cmd=cart_vel_cmd,
            deadzone=deadzone,
        )
        if np.linalg.norm(startup_escape) > 1e-9:
            candidates.append(('Z↑startup_escape', startup_escape))

        best_dq = dq
        best_label = 'Z↑blocked'
        best_delta = current_delta
        current_forward_error = self._forward_error_at_q(current_q)
        best_forward_ok = False

        for label, candidate in candidates:
            candidate = _clamp_joint_velocity_to_limits(
                current_q[:active_dofs],
                candidate[:active_dofs],
                self._q_lo[:active_dofs],
                self._q_hi[:active_dofs],
                self._dt,
            )
            candidate_delta = z_delta_for(candidate)
            if candidate_delta <= best_delta + 1e-5:
                continue

            candidate_forward_error = self._forward_error_at_q(next_q_for(candidate))
            forward_ok = (
                current_forward_error is None
                or candidate_forward_error is None
                or _forward_progress_is_acceptable(
                    current_forward_error,
                    candidate_forward_error,
                    tolerance=0.05,
                )
            )
            if forward_ok or not best_forward_ok:
                best_dq = candidate
                best_label = label
                best_delta = candidate_delta
                best_forward_ok = forward_ok

        if best_delta > 2e-4:
            return best_dq, [best_label]
        if pin_lower_elbow and np.linalg.norm(elbow_unstick) > 1e-9:
            elbow_unstick = _clamp_joint_velocity_to_limits(
                current_q[:active_dofs],
                elbow_unstick,
                self._q_lo[:active_dofs],
                self._q_hi[:active_dofs],
                self._dt,
            )
            return elbow_unstick, ['Z↑elbow_unstick']
        return np.zeros_like(dq), ['Z↑blocked']

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
            if cmd_norm > idle_threshold:
                current_q = self._q_continuous if self._q_continuous is not None else self._q
                self._startup_recovery_active = False
                self._startup_hold_complete = True
                self._startup_recovery_logged = False
                self._q_cmd = current_q.copy()
                self.get_logger().info('Startup recovery interrupted by manual arm command.')
            else:
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
                hold_target, hold_dq = _idle_hold_command(
                    current_q=self._q,
                    hold_target=hold_target,
                    q_lo=self._q_lo,
                    q_hi=self._q_hi,
                    dt=self._dt,
                    gain=hold_gain,
                    max_joint_vel=max_jv,
                    tolerance=hold_tolerance,
                    hold_armed=self._idle_hold_armed,
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
            if self._linear_mode:
                self._ee_pos_ref = None
            return

        # Solve from the live measured pose so the Jacobian tracks the actual arm.
        if not self._moving:
            self._moving = True
        self._idle_hold_armed = True
        solve_q = self._q_continuous if self._q_continuous is not None else self._q
        if self._ee_z_ref is None:
            self._ee_z_ref = float(self._get_ee_pos(solve_q)[2])

        # Control parameters.
        max_cv = self.get_parameter('max_cartesian_vel').get_parameter_value().double_value
        max_av = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        max_jv = self.get_parameter('max_joint_vel').get_parameter_value().double_value
        lam = self.get_parameter('jacobian_damping').get_parameter_value().double_value
        horizon = self.get_parameter('trajectory_horizon_sec').get_parameter_value().double_value
        horizon = max(horizon, self._dt * 2.0)  # never shorter than 2 control ticks

        J = self._get_jacobian(solve_q)
        if J is None:
            return

        active_dofs = J.shape[1]
        Jlin = J[:3, :active_dofs]
        cart_vel_cmd = self._cmd_vel * max_cv
        command_target_dt = horizon
        if abs(float(cart_vel_cmd[2])) >= deadzone:
            self._ee_z_ref = float(self._get_ee_pos(solve_q)[2])
        cart_vel = np.zeros(3)
        angular_vel = np.zeros(3)
        secondary_jacobian = None
        secondary_vel = None
        primary_jacobian = Jlin
        primary_vel = cart_vel
        forward_error_before = None
        forward_error_after = None
        mode_label = 'linear'
        startup_unfold_primary = False

        if self._linear_mode:
            # Interpret the input in the configured command frame, then solve in base coordinates.
            cart_vel = self._command_velocity_in_base(solve_q, cart_vel_cmd)
            cart_vel = _dominant_z_lift_velocity(
                cart_vel=cart_vel,
                current_q=solve_q,
                q_lo=self._q_lo,
                max_cartesian_vel=max_cv,
                cart_vel_cmd=cart_vel_cmd,
                deadzone=deadzone,
            )
            cart_vel = _linear_lateral_forward_velocity(
                cart_vel=cart_vel,
                cart_vel_cmd=cart_vel_cmd,
                max_cartesian_vel=max_cv,
                deadzone=deadzone,
                front_branch_gain=self._front_branch_gain,
            )
            command_target_dt = _linear_command_target_dt(
                command_mode=self._command_mode,
                horizon=horizon,
                control_dt=self._dt,
                cart_vel_cmd=cart_vel_cmd,
                deadzone=deadzone,
            )

            # Hold the current height unless the user is explicitly commanding Z.
            # This keeps lateral motion from slowly climbing as the arm changes posture.
            if self._ee_z_ref is not None and abs(cart_vel_cmd[2]) < deadzone:
                z_error = self._ee_z_ref - float(self._get_ee_pos(solve_q)[2])
                z_hold_vel = 2.0 * z_error
                hold_scale = _joint_limit_hold_scale(solve_q, self._q_lo, self._q_hi)
                # Relax the height hold near saturation so a reverse x/y command can
                # back the arm away from the limit instead of fighting the stale z target.
                cart_vel[2] = float(np.clip(z_hold_vel * hold_scale, -max_cv, max_cv))

            if _is_dominant_z_command(cart_vel_cmd, deadzone):
                linear_task_jacobian = Jlin[2:3, :]
                linear_task_vel = np.array([cart_vel[2]])
            else:
                linear_task_jacobian = Jlin
                linear_task_vel = cart_vel

            primary_jacobian = linear_task_jacobian
            primary_vel = linear_task_vel
            current_rotation = self._get_ee_rotation(solve_q)
            hold_orientation = True
            if current_rotation is not None and hold_orientation:
                hold_gain = (
                    self.get_parameter('orientation_hold_gain').get_parameter_value().double_value
                )
                orientation_task_weight = (
                    self.get_parameter('orientation_task_weight')
                    .get_parameter_value()
                    .double_value
                )
                angular_vel = _clip_vector(
                    hold_gain
                    * _rotation_error_vector(
                        current_rotation=current_rotation,
                        desired_rotation=self._forward_rotation,
                    ),
                    max_av,
                )
                if orientation_task_weight > 0.0:
                    secondary_jacobian, secondary_vel = _append_secondary_task(
                        secondary_jacobian=secondary_jacobian,
                        secondary_vel=secondary_vel,
                        task_jacobian=J[3:6, :active_dofs],
                        task_vel=angular_vel,
                        weight=orientation_task_weight,
                    )

            startup_escape_vel = _linear_startup_escape_velocity(
                current_q=solve_q[:active_dofs],
                q_lo=self._q_lo[:active_dofs],
                max_joint_vel=max_jv,
                cart_vel_cmd=cart_vel_cmd,
                deadzone=deadzone,
            )
            if np.linalg.norm(startup_escape_vel) > 1e-9:
                secondary_jacobian, secondary_vel = _append_secondary_task(
                    secondary_jacobian=secondary_jacobian,
                    secondary_vel=secondary_vel,
                    task_jacobian=np.eye(active_dofs),
                    task_vel=startup_escape_vel,
                    weight=0.8,
                )

            posture_weight = (
                self.get_parameter('linear_posture_weight').get_parameter_value().double_value
            )
            posture_gain = (
                self.get_parameter('linear_posture_gain').get_parameter_value().double_value
            )
            posture_vel = _clip_vector(
                posture_gain * (self._linear_posture_target[:active_dofs] - solve_q[:active_dofs]),
                0.35 * max_jv,
            )
            secondary_jacobian, secondary_vel = _append_secondary_task(
                secondary_jacobian=secondary_jacobian,
                secondary_vel=secondary_vel,
                task_jacobian=np.eye(active_dofs),
                task_vel=posture_vel,
                weight=posture_weight,
            )
        else:
            mode_label = 'rotation'
            if self._fk_solver is None:
                self.get_logger().warn(
                    'Rotation mode requires KDL; holding joint position instead.',
                    throttle_duration_sec=5.0,
                )
                self._publish_velocity([0.0] * self.N_JOINTS)
                return

            cart_vel = np.zeros(3)
            tool_angular_vel = _rotation_mode_angular_velocity(
                self._cmd_vel,
                max_av,
                deadzone=deadzone,
            )
            current_rotation = self._get_ee_rotation(solve_q)
            if current_rotation is None:
                angular_vel = tool_angular_vel
            else:
                # Roll: always around the camera's own forward axis (tool X in base frame).
                roll_axis = current_rotation[:, 0]
                # Tilt: always around the horizontal axis perpendicular to camera forward
                # (cross(camera_forward, world_Z)) so that up/down commands tilt the
                # camera up/down regardless of current arm pose.
                world_z = np.array([0.0, 0.0, 1.0])
                tilt_axis_raw = np.cross(world_z, roll_axis)
                tilt_norm = np.linalg.norm(tilt_axis_raw)
                tilt_axis = tilt_axis_raw / tilt_norm if tilt_norm > 0.1 else current_rotation[:, 1]
                # Yaw/pan: always around base-frame vertical Z.
                angular_vel = (
                    tool_angular_vel[0] * roll_axis
                    + tool_angular_vel[1] * tilt_axis
                    + np.array([0.0, 0.0, tool_angular_vel[2]])
                )
            # Angular velocity is the primary task.  The minimum-norm DLS solution
            # naturally routes pan to q1 (base_roll), keeping the motion intuitive.
            # Soft-stop pan when q1 approaches ±90° so the arm cannot swing to the rear.
            q1 = float(solve_q[1]) if len(solve_q) > 1 else 0.0
            pan_limit = np.pi / 2  # 90°
            pan_margin = 0.15      # start fading 0.15 rad before the limit
            pan_distance = pan_limit - abs(q1)
            pan_scale = float(np.clip(pan_distance / pan_margin, 0.0, 1.0))
            yaw_component = angular_vel[2]
            if (q1 > 0 and yaw_component < 0) or (q1 < 0 and yaw_component > 0):
                pan_scale = 1.0  # never suppress motion back toward centre
            angular_vel = angular_vel.copy()
            angular_vel[2] *= pan_scale
            primary_jacobian = J[3:6, :active_dofs]
            primary_vel = angular_vel

        limit_recovery = _joint_limit_recovery_velocity(
            current_q=solve_q,
            q_lo=self._q_lo,
            q_hi=self._q_hi,
            max_joint_vel=max_jv,
        )
        if self._linear_mode:
            secondary_jacobian, secondary_vel = _append_secondary_task(
                secondary_jacobian=secondary_jacobian,
                secondary_vel=secondary_vel,
                task_jacobian=np.eye(active_dofs),
                task_vel=limit_recovery[:active_dofs],
                weight=1.0,
            )

        # Damped least-squares IK on the active Cartesian task.
        try:
            joint_weights = self._joint_weights[:active_dofs]
            secondary_weight = (
                self.get_parameter('secondary_task_weight').get_parameter_value().double_value
            )
            if not self._linear_mode and active_dofs >= 6:
                # Rotation mode: solve ONLY with the 3×3 wrist sub-Jacobian so
                # that q0/q1/q2 are structurally excluded and never move.
                wrist_jacobian = J[3:6, 3:6]
                wrist_weights = self._joint_weights[3:6]
                wrist_mid = (self._q_lo[3:6] + self._q_hi[3:6]) / 2.0
                cmd_norm = float(np.linalg.norm(primary_vel))
                dq = np.zeros(active_dofs)
                if cmd_norm < 0.05:
                    # Joystick idle: recenter wrist joints toward their midpoints
                    # so the next command in either direction has full range.
                    center_vel = 0.3 * (wrist_mid - solve_q[3:6])
                    center_vel = np.clip(center_vel, -max_jv, max_jv)
                    dq[3:6] = center_vel
                    joints_clipped = []
                    vel_scale = 1.0
                else:
                    # Active rotation command: solve wrist-only.
                    # Blend in a gentle centering bias on q3/q5 so they don't
                    # accumulate at limits across repeated pan commands.
                    # The centering bias scales down to zero near the midpoint.
                    center_gain = 0.15
                    center_bias = center_gain * (wrist_mid - solve_q[3:6])
                    # Only apply centering on q3 and q5 (the ±π wrap joints).
                    # q4 (wrist_pitch) is asymmetric and naturally stays bounded.
                    center_vel_full = np.zeros(3)
                    center_vel_full[0] = center_bias[0]  # q3
                    center_vel_full[2] = center_bias[2]  # q5
                    # Modified cart_vel = commanded + centering projected through J.
                    # Instead of modifying cart_vel (which would corrupt the angular
                    # command), inject centering directly into dq after the solve.
                    dq_wrist, joints_clipped, vel_scale = _solve_task_velocity_with_limit_redistribution(
                        current_q=solve_q[3:6],
                        task_jacobian=wrist_jacobian,
                        cart_vel=primary_vel,
                        q_lo=self._q_lo[3:6],
                        q_hi=self._q_hi[3:6],
                        dt=self._dt,
                        max_joint_vel=max_jv,
                        damping=lam,
                        joint_weights=wrist_weights,
                    )
                    # Add centering bias in the nullspace of the 3×3 wrist Jacobian.
                    # The 3×3 wrist Jacobian is square (no nullspace), so we use
                    # a weighted addition that diminishes with command magnitude.
                    # At full stick the bias is negligible; near zero it ramps up.
                    bias_scale = float(np.clip(1.0 - cmd_norm / 0.8, 0.0, 1.0))
                    dq_wrist = dq_wrist + bias_scale * center_vel_full
                    dq_wrist = np.clip(dq_wrist, -max_jv, max_jv)
                    dq[3:6] = dq_wrist
            else:
                dq, joints_clipped, vel_scale = _solve_prioritized_task_velocity(
                    current_q=solve_q,
                    primary_jacobian=primary_jacobian,
                    primary_vel=primary_vel,
                    secondary_jacobian=secondary_jacobian,
                    secondary_vel=secondary_vel,
                    q_lo=self._q_lo,
                    q_hi=self._q_hi,
                    dt=self._dt,
                    max_joint_vel=max_jv,
                    damping=lam,
                    joint_weights=joint_weights,
                    secondary_weight=secondary_weight,
                )
        except np.linalg.LinAlgError:
            self.get_logger().warn('DLS solve failed.')
            return

        if (
            self._linear_mode
            and not startup_unfold_primary
            and _is_dominant_z_command(cart_vel_cmd, deadzone)
        ):
            max_xy_leak = max(0.04, 0.15 * abs(float(cart_vel[2])))
            dq = _limit_task_space_velocity(
                dq=dq[:active_dofs],
                task_jacobian=Jlin[0:2, :],
                max_task_speed=max_xy_leak,
                joint_weights=joint_weights,
                damping=min(lam, 1e-4),
            )
            dq = _clamp_joint_velocity_to_limits(
                solve_q[:active_dofs],
                dq,
                self._q_lo[:active_dofs],
                self._q_hi[:active_dofs],
                self._dt,
            )

        if (
            self._linear_mode
            and not startup_unfold_primary
            and _is_dominant_z_command(cart_vel_cmd, deadzone)
        ):
            z_jacobian = Jlin[2:3, :]
            desired_z_vel = float(cart_vel[2])
            achieved_z_vel = float((z_jacobian @ dq[:active_dofs])[0])
            min_z_progress = 0.0
            if _is_dominant_positive_z_command(cart_vel_cmd, deadzone):
                min_z_progress = _minimum_positive_z_progress(desired_z_vel)
            z_error = achieved_z_vel - desired_z_vel
            if abs(z_error) > 1e-4:
                z_correction = _solve_weighted_dls_task_velocity(
                    task_jacobian=z_jacobian,
                    cart_vel=np.array([z_error]),
                    joint_weights=joint_weights,
                    damping=min(lam, 1e-4),
                )
                dq = _clamp_joint_velocity_to_limits(
                    solve_q[:active_dofs],
                    dq[:active_dofs] - z_correction,
                    self._q_lo[:active_dofs],
                    self._q_hi[:active_dofs],
                    self._dt,
                )
                achieved_z_vel = float((z_jacobian @ dq[:active_dofs])[0])

            if (
                abs(desired_z_vel) > deadzone
                and (
                    achieved_z_vel * desired_z_vel <= 0.0
                    or (
                        _is_dominant_positive_z_command(cart_vel_cmd, deadzone)
                        and achieved_z_vel < min_z_progress
                    )
                )
            ):
                z_only_dq, z_only_clipped, z_only_scale = _solve_directional_z_velocity(
                    current_q=solve_q[:active_dofs],
                    z_jacobian=z_jacobian,
                    desired_z_vel=desired_z_vel,
                    q_lo=self._q_lo[:active_dofs],
                    q_hi=self._q_hi[:active_dofs],
                    dt=self._dt,
                    max_joint_vel=max_jv,
                    damping=lam,
                    joint_weights=joint_weights,
                )
                z_only_vel = float((z_jacobian @ z_only_dq[:active_dofs])[0])
                if (
                    z_only_vel * desired_z_vel > 0.0
                ):
                    dq = z_only_dq
                    joints_clipped = [*joints_clipped, *z_only_clipped, 'Z↕guard']
                    vel_scale = min(vel_scale, z_only_scale)
                else:
                    dq = np.zeros_like(dq)
                    joints_clipped = [*joints_clipped, 'Z↕blocked']

        if (
            self._linear_mode
            and not startup_unfold_primary
            and _is_lateral_forward_bias_active(cart_vel_cmd, deadzone)
        ):
            x_jacobian = Jlin[0:1, :]
            desired_x_vel = max(float(cart_vel[0]), 0.0)
            achieved_x_vel = float((x_jacobian @ dq[:active_dofs])[0])
            x_error = achieved_x_vel - desired_x_vel
            if x_error < -1e-4:
                x_correction = _solve_weighted_dls_task_velocity(
                    task_jacobian=x_jacobian,
                    cart_vel=np.array([x_error]),
                    joint_weights=joint_weights,
                    damping=min(lam, 1e-4),
                )
                dq = _clamp_joint_velocity_to_limits(
                    solve_q[:active_dofs],
                    dq[:active_dofs] - x_correction,
                    self._q_lo[:active_dofs],
                    self._q_hi[:active_dofs],
                    self._dt,
                )
                achieved_x_vel = float((x_jacobian @ dq[:active_dofs])[0])

            if desired_x_vel > deadzone and achieved_x_vel <= 0.0:
                x_only_dq, x_only_clipped, x_only_scale = _solve_task_velocity_with_limit_redistribution(
                    current_q=solve_q[:active_dofs],
                    task_jacobian=x_jacobian,
                    cart_vel=np.array([desired_x_vel]),
                    q_lo=self._q_lo[:active_dofs],
                    q_hi=self._q_hi[:active_dofs],
                    dt=self._dt,
                    max_joint_vel=max_jv,
                    damping=lam,
                    joint_weights=joint_weights,
                )
                x_only_vel = float((x_jacobian @ x_only_dq[:active_dofs])[0])
                if x_only_vel > 0.0:
                    dq = x_only_dq
                    joints_clipped = [*joints_clipped, *x_only_clipped, 'X→guard']
                    vel_scale = min(vel_scale, x_only_scale)
                else:
                    dq = np.zeros_like(dq)
                    joints_clipped = [*joints_clipped, 'X→blocked']

        if (
            self._linear_mode
            and not startup_unfold_primary
            and _is_dominant_positive_z_command(cart_vel_cmd, deadzone)
        ):
            current_z = float(self._get_ee_pos(solve_q)[2])
            next_q = np.clip(solve_q + dq * self._dt, self._q_lo, self._q_hi)
            next_z = float(self._get_ee_pos(next_q)[2])
            if next_z < current_z - 1e-4:
                rescue_dq = _linear_startup_escape_velocity(
                    current_q=solve_q[:active_dofs],
                    q_lo=self._q_lo[:active_dofs],
                    max_joint_vel=max_jv,
                    cart_vel_cmd=cart_vel_cmd,
                    deadzone=deadzone,
                )
                if np.linalg.norm(rescue_dq) > 1e-9:
                    rescue_dq = _clamp_joint_velocity_to_limits(
                        solve_q[:active_dofs],
                        rescue_dq,
                        self._q_lo[:active_dofs],
                        self._q_hi[:active_dofs],
                        self._dt,
                    )
                    rescue_next_q = solve_q.copy()
                    rescue_next_q[:active_dofs] = np.clip(
                        solve_q[:active_dofs] + rescue_dq * self._dt,
                        self._q_lo[:active_dofs],
                        self._q_hi[:active_dofs],
                    )
                    rescue_next_z = float(self._get_ee_pos(rescue_next_q)[2])
                    if rescue_next_z >= current_z - 1e-4:
                        dq = rescue_dq
                        joints_clipped = [*joints_clipped, 'Z↑guard']
                    else:
                        dq = np.zeros_like(dq)
                        joints_clipped = [*joints_clipped, 'Z↑blocked']
                else:
                    dq = np.zeros_like(dq)
                    joints_clipped = [*joints_clipped, 'Z↑blocked']

        if self._linear_mode:
            (
                dq,
                forward_guard_clipped,
                forward_guard_scale,
                forward_error_before,
                forward_error_after,
            ) = self._enforce_linear_forward_progress(
                current_q=solve_q,
                dq=dq[:active_dofs],
                angular_jacobian=J[3:6, :active_dofs],
                angular_vel=angular_vel,
                linear_task_jacobian=linear_task_jacobian,
                linear_task_vel=linear_task_vel,
                joint_weights=joint_weights,
                max_joint_vel=max_jv,
                damping=lam,
                deadzone=deadzone,
                active_dofs=active_dofs,
            )
            if forward_guard_clipped:
                joints_clipped = [*joints_clipped, *forward_guard_clipped]
                vel_scale = min(vel_scale, forward_guard_scale)

            (
                dq,
                positive_z_fk_clipped,
            ) = self._enforce_positive_z_fk_progress(
                current_q=solve_q,
                dq=dq[:active_dofs],
                cart_vel_cmd=cart_vel_cmd,
                deadzone=deadzone,
                max_joint_vel=max_jv,
                active_dofs=active_dofs,
            )
            if positive_z_fk_clipped:
                joints_clipped = [*joints_clipped, *positive_z_fk_clipped]
                if positive_z_fk_clipped == ['Z↑blocked']:
                    vel_scale = 0.0

        # In trajectory mode, generate the next target from the measured pose.
        # This keeps the published joint step consistent with the Jacobian
        # linearization and avoids accumulating backlog when hardware tracking
        # lags behind the previously commanded target.
        if self._command_mode == 'trajectory':
            self._q_cmd = np.clip(solve_q + dq * command_target_dt, self._q_lo, self._q_hi)
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
            achieved_task = Jlin @ dq[:active_dofs]
            orientation_diag = ''
            if forward_error_before is not None and forward_error_after is not None:
                orientation_diag = (
                    f'orientation_error={forward_error_before:.3f}->{forward_error_after:.3f} '
                )
            self.get_logger().info(
                f'mode={mode_label} '
                f'cart_in={np.round(cart_vel, 3)} '
                f'angular_in={np.round(angular_vel, 3)} '
                f'dq={np.round(dq, 3)} '
                f'linear_out={np.round(achieved_task, 3)} '
                f'vel_scale={vel_scale:.2f} '
                f'{orientation_diag}'
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
        self._idle_hold_armed = True
        self._cmd_vel = np.zeros(3)
        self._ee_z_ref = None
        self._ee_pos_ref = None
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
        self._ee_z_ref = None
        if req.data:
            self._ee_pos_ref = None
        elif self._q is not None:
            current_q = self._q_continuous if self._q_continuous is not None else self._q
            self._ee_pos_ref = self._get_ee_pos(current_q).copy()
        res.success = True
        res.message = 'End-effector mode -> LINEAR' if req.data else 'End-effector mode -> ROTATION'
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
