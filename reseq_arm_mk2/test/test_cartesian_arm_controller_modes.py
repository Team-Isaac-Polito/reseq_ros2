import numpy as np

from reseq_arm_mk2.cartesian_arm_controller import (
    _dominant_z_lift_velocity,
    _forward_axis_error_vector,
    _is_dominant_positive_z_command,
    _is_dominant_z_command,
    _linear_startup_escape_velocity,
    _remove_task_space_velocity,
    _rotation_error_vector,
    _rotation_matrix_from_rpy,
    _rotation_mode_angular_velocity,
    _solve_prioritized_task_velocity,
    _solve_task_velocity_with_limit_redistribution,
)


def test_rotation_mode_maps_scaler_xyz_to_roll_tilt_pan():
    angular = _rotation_mode_angular_velocity(
        cmd_vel=np.array([1.0, 0.5, -0.25]),
        max_angular_vel=0.8,
    )

    assert np.allclose(angular, np.array([0.8, -0.2, 0.4]))


def test_rotation_error_uses_fixed_robot_forward_orientation():
    current = _rotation_matrix_from_rpy(0.0, 0.0, 0.25)
    desired = _rotation_matrix_from_rpy(0.0, 0.0, 0.0)

    error = _rotation_error_vector(
        current_rotation=current,
        desired_rotation=desired,
    )

    assert np.allclose(error, np.array([0.0, 0.0, -0.25]), atol=1e-6)


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


def test_dominant_positive_z_command_filters_normal_xy_stick_motion():
    assert _is_dominant_positive_z_command(np.array([0.0, 0.0, 0.2]), 0.02)
    assert not _is_dominant_positive_z_command(np.array([0.2, 0.0, 0.05]), 0.02)
    assert not _is_dominant_positive_z_command(np.array([0.0, 0.0, -0.2]), 0.02)


def test_dominant_z_command_accepts_up_and_down_vertical_commands():
    assert _is_dominant_z_command(np.array([0.0, 0.0, 0.2]), 0.02)
    assert _is_dominant_z_command(np.array([0.0, 0.0, -0.2]), 0.02)
    assert not _is_dominant_z_command(np.array([0.2, 0.0, 0.05]), 0.02)


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
