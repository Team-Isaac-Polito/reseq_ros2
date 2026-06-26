import numpy as np

from reseq_arm_mk2.cartesian_arm_controller import (
    _dominant_z_lift_velocity,
    _forward_axis_alignment_error,
    _forward_axis_error_vector,
    _forward_progress_is_acceptable,
    _is_dominant_positive_z_command,
    _is_dominant_z_command,
    _is_lateral_forward_bias_active,
    _is_lower_elbow_positive_z_recovery_active,
    _is_lower_elbow_hard_corner,
    _linear_command_target_dt,
    _linear_orientation_hold_weight,
    _linear_lateral_forward_velocity,
    _linear_startup_escape_velocity,
    _lower_elbow_command_spends_stop_reserve,
    _lower_elbow_corner_unstick_velocity,
    _lower_elbow_positive_z_escape_velocity,
    _minimum_positive_z_progress,
    _remove_task_space_velocity,
    _rotation_error_vector,
    _rotation_matrix_from_rpy,
    _rotation_mode_angular_velocity,
    _solve_directional_z_velocity,
    _solve_prioritized_task_velocity,
    _solve_task_velocity_with_limit_redistribution,
    _task_velocity_direction_is_acceptable,
)


def test_rotation_mode_maps_scaler_xyz_to_roll_tilt_pan():
    angular = _rotation_mode_angular_velocity(
        cmd_vel=np.array([1.0, 0.5, -0.25]),
        max_angular_vel=0.8,
    )

    assert np.allclose(angular, np.array([0.8, 0.2, 0.4]))


def test_rotation_mode_yaw_is_around_base_z_not_tool_z():
    # Tool pitched 90° about Y: tool Z now points along -base X.
    # Before the fix, a left/right command would rotate around base X (wrong).
    # With the fix it must produce rotation around base Z only.
    tool_rotation = _rotation_matrix_from_rpy(0.0, np.pi / 2, 0.0)

    # Pure left/right joystick: _rotation_mode_angular_velocity maps cmd_vel[1] -> tool_angular_vel[2]
    max_av = 0.8
    tool_angular_vel = np.array([0.0, 0.0, max_av])

    roll_tilt_tool = np.array([tool_angular_vel[0], tool_angular_vel[1], 0.0])
    yaw_base = np.array([0.0, 0.0, tool_angular_vel[2]])
    angular_vel = tool_rotation @ roll_tilt_tool + yaw_base

    assert np.isclose(angular_vel[2], max_av)
    assert np.isclose(angular_vel[0], 0.0, atol=1e-9)
    assert np.isclose(angular_vel[1], 0.0, atol=1e-9)


def test_rotation_error_uses_fixed_robot_forward_orientation():
    current = _rotation_matrix_from_rpy(0.0, 0.0, 0.25)
    desired = _rotation_matrix_from_rpy(0.0, 0.0, 0.0)

    error = _rotation_error_vector(
        current_rotation=current,
        desired_rotation=desired,
    )

    assert np.allclose(error, np.array([0.0, 0.0, -0.25]), atol=1e-6)


def test_rotation_error_corrects_roll_about_forward_axis():
    current = _rotation_matrix_from_rpy(0.4, 0.0, 0.0)
    desired = _rotation_matrix_from_rpy(0.0, 0.0, 0.0)

    error = _rotation_error_vector(
        current_rotation=current,
        desired_rotation=desired,
    )

    assert error[0] < 0.0
    assert np.allclose(error[1:], np.zeros(2), atol=1e-6)


def test_rotation_error_is_zero_when_camera_is_forward():
    forward = _rotation_matrix_from_rpy(0.0, 0.0, 0.0)

    error = _rotation_error_vector(
        current_rotation=forward,
        desired_rotation=forward,
    )

    assert np.allclose(error, np.zeros(3))


def test_forward_axis_error_ignores_roll_about_forward_axis():
    current = _rotation_matrix_from_rpy(0.8, 0.0, 0.0)
    desired = _rotation_matrix_from_rpy(0.0, 0.0, 0.0)

    error = _forward_axis_error_vector(
        current_rotation=current,
        desired_rotation=desired,
    )

    assert np.allclose(error, np.zeros(3), atol=1e-6)


def test_forward_axis_error_corrects_sideways_tool_axis():
    current = _rotation_matrix_from_rpy(0.0, 0.0, 0.5)
    desired = _rotation_matrix_from_rpy(0.0, 0.0, 0.0)

    error = _forward_axis_error_vector(
        current_rotation=current,
        desired_rotation=desired,
    )

    assert error[2] < 0.0


def test_forward_axis_alignment_error_measures_axis_angle():
    current = _rotation_matrix_from_rpy(0.0, 0.0, 0.5)
    desired = _rotation_matrix_from_rpy(0.0, 0.0, 0.0)

    error = _forward_axis_alignment_error(current, desired)

    assert np.isclose(error, 0.5)


def test_forward_progress_requires_recovery_when_not_forward():
    assert _forward_progress_is_acceptable(0.5, 0.49)
    assert not _forward_progress_is_acceptable(0.5, 0.5)
    assert not _forward_progress_is_acceptable(0.5, 0.51)


def test_forward_progress_allows_small_error_growth_inside_tolerance():
    assert _forward_progress_is_acceptable(0.003, 0.015)
    assert not _forward_progress_is_acceptable(0.003, 0.03)


def test_linear_orientation_hold_weight_is_disabled_when_far_from_forward():
    assert _linear_orientation_hold_weight(None, 1.4) == 0.0
    assert _linear_orientation_hold_weight(1.2, 1.4) == 0.0
    assert np.isclose(_linear_orientation_hold_weight(0.35, 1.4), 1.4)
    mid_weight = _linear_orientation_hold_weight(0.625, 1.4)
    assert 0.0 < mid_weight < 1.4


def test_task_velocity_direction_rejects_reversed_dominant_axis():
    assert _task_velocity_direction_is_acceptable(
        desired_vel=np.array([0.0, -0.4, 0.0]),
        achieved_vel=np.array([0.1, -0.02, 0.1]),
        deadzone=0.02,
    )
    assert not _task_velocity_direction_is_acceptable(
        desired_vel=np.array([0.0, -0.4, 0.0]),
        achieved_vel=np.array([0.1, 0.02, 0.1]),
        deadzone=0.02,
    )


def test_startup_z_escape_biases_folded_arm_out_of_lower_elbow_limit():
    escape = _linear_startup_escape_velocity(
        current_q=np.array([-0.03, 0.0, -0.1, 0.0, 0.0, 0.0]),
        q_lo=np.array([-0.1, -3.14, -0.1, -3.14, -0.46, -3.14]),
        max_joint_vel=0.8,
        cart_vel_cmd=np.array([0.0, 0.0, 0.2]),
        deadzone=0.02,
    )

    assert escape[0] < 0.0
    assert escape[2] > 0.0
    assert escape[4] < 0.0
    assert abs(escape[2]) > abs(escape[0])


def test_startup_z_escape_stops_pushing_wrist_at_lower_limit():
    escape = _linear_startup_escape_velocity(
        current_q=np.array([0.5, 0.0, -0.1, 0.0, -0.46, 0.0]),
        q_lo=np.array([-0.1, -3.14, -0.1, -3.14, -0.46, -3.14]),
        max_joint_vel=0.8,
        cart_vel_cmd=np.array([0.0, 0.0, 0.2]),
        deadzone=0.02,
    )

    assert escape[0] < 0.0
    assert escape[2] > 0.0
    assert escape[4] == 0.0


def test_startup_z_escape_ignores_non_dominant_z_drift():
    escape = _linear_startup_escape_velocity(
        current_q=np.array([-0.03, 0.0, -0.1, 0.0, 0.0, 0.0]),
        q_lo=np.array([-0.1, -3.14, -0.1, -3.14, -0.46, -3.14]),
        max_joint_vel=0.8,
        cart_vel_cmd=np.array([0.2, 0.0, 0.05]),
        deadzone=0.02,
    )

    assert np.allclose(escape, np.zeros(6))


def test_startup_z_escape_is_inactive_after_unfolding():
    escape = _linear_startup_escape_velocity(
        current_q=np.array([0.5, 0.0, 0.3, 0.0, 0.0, 0.0]),
        q_lo=np.array([-0.1, -3.14, -0.1, -3.14, -0.46, -3.14]),
        max_joint_vel=0.8,
        cart_vel_cmd=np.array([0.0, 0.0, 0.2]),
        deadzone=0.02,
    )

    assert np.allclose(escape, np.zeros(6))


def test_lower_elbow_positive_z_recovery_detects_folded_lift_trap():
    assert _is_lower_elbow_positive_z_recovery_active(
        current_q=np.array([0.3, 0.0, -0.1, 0.0, -0.08, 0.0]),
        q_lo=np.array([-0.1, -3.14, -0.1, -3.14, -0.46, -3.14]),
        cart_vel_cmd=np.array([0.0, 0.0, 0.6]),
        deadzone=0.02,
    )
    assert not _is_lower_elbow_positive_z_recovery_active(
        current_q=np.array([0.3, 0.0, 0.1, 0.0, -0.08, 0.0]),
        q_lo=np.array([-0.1, -3.14, -0.1, -3.14, -0.46, -3.14]),
        cart_vel_cmd=np.array([0.0, 0.0, 0.6]),
        deadzone=0.02,
    )


def test_lower_elbow_positive_z_escape_uses_shoulder_wrist_and_elbow():
    escape = _lower_elbow_positive_z_escape_velocity(
        current_q=np.array([0.3, 0.0, -0.1, 0.0, -0.08, 0.0]),
        q_lo=np.array([-0.1, -3.14, -0.1, -3.14, -0.46, -3.14]),
        q_hi=np.array([2.8, 3.14, 2.88, 3.14, 1.57, 3.14]),
        max_joint_vel=1.6,
    )

    assert escape[0] < 0.0
    assert escape[2] > 0.0
    assert escape[4] < 0.0


def test_lower_elbow_positive_z_escape_preserves_stop_reserve():
    escape = _lower_elbow_positive_z_escape_velocity(
        current_q=np.array([0.04, 0.0, -0.1, 0.0, -0.35, 0.0]),
        q_lo=np.array([-0.1, -3.14, -0.1, -3.14, -0.46, -3.14]),
        q_hi=np.array([2.8, 3.14, 2.88, 3.14, 1.57, 3.14]),
        max_joint_vel=1.6,
    )

    assert escape[0] == 0.0
    assert escape[2] > 0.0
    assert escape[4] == 0.0


def test_lower_elbow_corner_unstick_opens_hard_stop_pose():
    unstick = _lower_elbow_corner_unstick_velocity(
        current_q=np.array([-0.1, 0.0, -0.1, 0.0, -0.447, 0.0]),
        q_lo=np.array([-0.1, -3.14, -0.1, -3.14, -0.46, -3.14]),
        q_hi=np.array([2.8, 3.14, 2.88, 3.14, 1.57, 3.14]),
        max_joint_vel=1.6,
    )

    assert unstick[0] > 0.0
    assert unstick[2] > 0.0
    assert unstick[4] > 0.0


def test_lower_elbow_command_spends_stop_reserve_detects_tighter_tuck():
    q_lo = np.array([-0.1, -3.14, -0.1, -3.14, -0.46, -3.14])

    assert _lower_elbow_command_spends_stop_reserve(
        current_q=np.array([0.02, 0.0, -0.1, 0.0, -0.32, 0.0]),
        dq=np.array([-0.2, 0.0, 1.0, 0.0, 0.0, 0.0]),
        q_lo=q_lo,
    )
    assert _lower_elbow_command_spends_stop_reserve(
        current_q=np.array([0.2, 0.0, -0.1, 0.0, -0.34, 0.0]),
        dq=np.array([0.0, 0.0, 1.0, 0.0, -0.2, 0.0]),
        q_lo=q_lo,
    )
    assert not _lower_elbow_command_spends_stop_reserve(
        current_q=np.array([0.2, 0.0, -0.1, 0.0, -0.1, 0.0]),
        dq=np.array([-0.2, 0.0, 1.0, 0.0, -0.2, 0.0]),
        q_lo=q_lo,
    )


def test_lower_elbow_hard_corner_detects_exact_stuck_pose():
    q_lo = np.array([-0.1, -3.14, -0.1, -3.14, -0.46, -3.14])

    assert _is_lower_elbow_hard_corner(
        current_q=np.array([-0.1, 0.0, -0.1, 0.0, -0.447, 0.0]),
        q_lo=q_lo,
    )
    assert not _is_lower_elbow_hard_corner(
        current_q=np.array([0.02, 0.0, -0.1, 0.0, -0.277, 0.0]),
        q_lo=q_lo,
    )


def test_dominant_positive_z_command_filters_normal_xy_stick_motion():
    assert _is_dominant_positive_z_command(np.array([0.0, 0.0, 0.2]), 0.02)
    assert not _is_dominant_positive_z_command(np.array([0.2, 0.0, 0.05]), 0.02)
    assert not _is_dominant_positive_z_command(np.array([0.0, 0.0, -0.2]), 0.02)


def test_dominant_z_command_accepts_up_and_down_vertical_commands():
    assert _is_dominant_z_command(np.array([0.0, 0.0, 0.2]), 0.02)
    assert _is_dominant_z_command(np.array([0.0, 0.0, -0.2]), 0.02)
    assert not _is_dominant_z_command(np.array([0.2, 0.0, 0.05]), 0.02)


def test_positive_z_trajectory_target_uses_control_tick():
    assert np.isclose(
        _linear_command_target_dt(
            command_mode='trajectory',
            horizon=0.1,
            control_dt=1.0 / 33.0,
            cart_vel_cmd=np.array([0.0, 0.0, 0.6]),
            deadzone=0.02,
        ),
        1.0 / 33.0,
    )
    assert np.isclose(
        _linear_command_target_dt(
            command_mode='trajectory',
            horizon=0.1,
            control_dt=1.0 / 33.0,
            cart_vel_cmd=np.array([0.0, 0.0, -0.4]),
            deadzone=0.02,
        ),
        0.1,
    )
    assert np.isclose(
        _linear_command_target_dt(
            command_mode='trajectory',
            horizon=0.1,
            control_dt=1.0 / 33.0,
            cart_vel_cmd=np.array([0.3, 0.0, 0.05]),
            deadzone=0.02,
        ),
        0.1,
    )


def test_dominant_z_lift_boosts_vertical_without_forward_escape():
    lifted = _dominant_z_lift_velocity(
        cart_vel=np.array([0.0, 0.0, 0.2]),
        current_q=np.array([-0.03, 0.0, -0.1, 0.0, 0.0, 0.0]),
        q_lo=np.array([-0.1, -3.14, -0.1, -3.14, -0.46, -3.14]),
        max_cartesian_vel=0.4,
        cart_vel_cmd=np.array([0.0, 0.0, 0.2]),
        deadzone=0.02,
    )

    assert lifted[0] == 0.0
    assert lifted[2] > 0.2


def test_dominant_z_lift_ignores_xy_dominant_motion():
    lifted = _dominant_z_lift_velocity(
        cart_vel=np.array([0.2, 0.0, 0.05]),
        current_q=np.array([-0.03, 0.0, -0.1, 0.0, 0.0, 0.0]),
        q_lo=np.array([-0.1, -3.14, -0.1, -3.14, -0.46, -3.14]),
        max_cartesian_vel=0.4,
        cart_vel_cmd=np.array([0.2, 0.0, 0.05]),
        deadzone=0.02,
    )

    assert np.allclose(lifted, np.array([0.2, 0.0, 0.05]))


def test_lateral_linear_command_does_not_inject_forward_bias():
    cart_vel = _linear_lateral_forward_velocity(
        cart_vel=np.array([0.0, -0.4, 0.0]),
        cart_vel_cmd=np.array([0.0, -0.4, 0.0]),
        max_cartesian_vel=0.6,
        deadzone=0.02,
        front_branch_gain=0.05,
    )

    assert np.allclose(cart_vel, np.array([0.0, -0.4, 0.0]))
    assert not _is_lateral_forward_bias_active(np.array([0.0, -0.4, 0.0]), 0.02)


def test_directional_z_solver_uses_only_joints_that_lift_up():
    dq, clipped, scale = _solve_directional_z_velocity(
        current_q=np.array([-0.1, -0.1, 0.0]),
        z_jacobian=np.array([[-0.2, 0.35, -0.15]]),
        desired_z_vel=0.3,
        q_lo=np.array([-0.1, -0.1, -1.0]),
        q_hi=np.array([1.0, 1.0, 1.0]),
        dt=0.1,
        max_joint_vel=1.6,
        damping=1e-6,
        joint_weights=np.ones(3),
    )

    assert clipped == ['J0↓']
    assert scale == 1.0
    assert dq[0] == 0.0
    assert dq[1] > 0.0
    assert dq[2] < 0.0
    assert float((np.array([[-0.2, 0.35, -0.15]]) @ dq)[0]) > 0.25


def test_positive_z_progress_threshold_scales_with_command_size():
    assert _minimum_positive_z_progress(0.027) < 0.027
    assert np.isclose(_minimum_positive_z_progress(0.6), 0.15)


def test_linear_primary_translation_is_not_changed_by_secondary_task():
    dq, clipped_joints, vel_scale = _solve_prioritized_task_velocity(
        current_q=np.zeros(2),
        primary_jacobian=np.array([[1.0, 0.0]]),
        primary_vel=np.array([0.15]),
        secondary_jacobian=np.eye(2),
        secondary_vel=np.array([0.0, -0.2]),
        q_lo=np.array([-1.0, -1.0]),
        q_hi=np.array([1.0, 1.0]),
        dt=0.1,
        max_joint_vel=1.0,
        damping=1e-6,
        joint_weights=np.ones(2),
        secondary_weight=1.0,
    )

    assert clipped_joints == []
    assert vel_scale == 1.0
    assert np.allclose(np.array([[1.0, 0.0]]) @ dq, np.array([0.15]), atol=1e-6)
    assert dq[1] < 0.0


def test_rotation_primary_keeps_linear_velocity_zero():
    linear_jacobian = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0],
        ]
    )
    angular_jacobian = np.array([[0.0, 0.0, 1.0]])
    primary_jacobian = np.vstack((linear_jacobian, angular_jacobian))

    dq, clipped_joints, vel_scale = _solve_prioritized_task_velocity(
        current_q=np.zeros(3),
        primary_jacobian=primary_jacobian,
        primary_vel=np.array([0.0, 0.0, 0.0, 0.4]),
        secondary_jacobian=None,
        secondary_vel=None,
        q_lo=np.array([-1.0, -1.0, -1.0]),
        q_hi=np.array([1.0, 1.0, 1.0]),
        dt=0.1,
        max_joint_vel=1.0,
        damping=1e-6,
        joint_weights=np.ones(3),
        secondary_weight=1.0,
    )

    assert clipped_joints == []
    assert vel_scale == 1.0
    assert np.allclose(linear_jacobian @ dq, np.zeros(3), atol=1e-6)
    assert np.allclose(angular_jacobian @ dq, np.array([0.4]), atol=1e-6)


def test_rotation_projection_removes_linear_drift_from_angular_task():
    linear_jacobian = np.array(
        [
            [1.0, 0.2, 0.0],
            [0.0, 1.0, 0.1],
            [0.3, 0.0, 1.0],
        ]
    )
    drifting_dq = np.array([0.12, -0.08, 0.2])

    corrected_dq = _remove_task_space_velocity(
        dq=drifting_dq,
        task_jacobian=linear_jacobian,
        joint_weights=np.ones(3),
        damping=1e-9,
    )

    assert np.linalg.norm(linear_jacobian @ corrected_dq) < 1e-6


def test_secondary_task_uses_leftover_joint_velocity_budget():
    primary_jacobian = np.array([[1.0, 0.0]])

    dq, clipped_joints, vel_scale = _solve_prioritized_task_velocity(
        current_q=np.zeros(2),
        primary_jacobian=primary_jacobian,
        primary_vel=np.array([1.0]),
        secondary_jacobian=np.array([[0.0, 1.0]]),
        secondary_vel=np.array([2.0]),
        q_lo=np.array([-2.0, -2.0]),
        q_hi=np.array([2.0, 2.0]),
        dt=0.1,
        max_joint_vel=1.0,
        damping=1e-6,
        joint_weights=np.ones(2),
        secondary_weight=1.0,
    )

    assert clipped_joints == []
    assert vel_scale == 1.0
    assert np.allclose(primary_jacobian @ dq, np.array([1.0]), atol=1e-6)
    assert dq[1] <= 1.0
    assert dq[1] > 0.0


def test_linear_posture_secondary_escapes_when_primary_direction_is_singular():
    dq, clipped_joints, vel_scale = _solve_prioritized_task_velocity(
        current_q=np.array([-0.1, 0.0]),
        primary_jacobian=np.zeros((1, 2)),
        primary_vel=np.array([0.2]),
        secondary_jacobian=np.eye(2),
        secondary_vel=np.array([0.3, 0.0]),
        q_lo=np.array([-0.1, -1.0]),
        q_hi=np.array([1.0, 1.0]),
        dt=0.1,
        max_joint_vel=1.0,
        damping=1e-6,
        joint_weights=np.ones(2),
        secondary_weight=1.0,
    )

    assert clipped_joints == []
    assert vel_scale == 1.0
    assert dq[0] > 0.0


def test_orientation_secondary_reduces_error_without_overriding_translation():
    primary_jacobian = np.array([[1.0, 0.0]])
    secondary_jacobian = np.array([[1.0, 1.0]])

    dq, clipped_joints, vel_scale = _solve_prioritized_task_velocity(
        current_q=np.zeros(2),
        primary_jacobian=primary_jacobian,
        primary_vel=np.array([0.1]),
        secondary_jacobian=secondary_jacobian,
        secondary_vel=np.array([0.0]),
        q_lo=np.array([-1.0, -1.0]),
        q_hi=np.array([1.0, 1.0]),
        dt=0.1,
        max_joint_vel=1.0,
        damping=1e-6,
        joint_weights=np.ones(2),
        secondary_weight=1.0,
    )

    assert clipped_joints == []
    assert vel_scale == 1.0
    assert np.allclose(primary_jacobian @ dq, np.array([0.1]), atol=1e-6)
    assert abs((secondary_jacobian @ dq)[0]) < 0.02


def test_joint_limit_recovery_is_not_added_to_primary_task_directly():
    dq, clipped_joints, vel_scale = _solve_task_velocity_with_limit_redistribution(
        current_q=np.array([0.0, -0.95]),
        task_jacobian=np.array([[1.0, 0.0]]),
        cart_vel=np.array([0.2]),
        q_lo=np.array([-1.0, -1.0]),
        q_hi=np.array([1.0, 1.0]),
        dt=0.1,
        max_joint_vel=1.0,
        damping=1e-6,
        joint_weights=np.ones(2),
    )

    assert clipped_joints == []
    assert vel_scale == 1.0
    assert np.allclose(dq, np.array([0.2, 0.0]), atol=1e-6)
