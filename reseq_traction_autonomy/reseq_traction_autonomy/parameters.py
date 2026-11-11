"""Default ROS parameters of the traction manager. Override them in the launch file."""

from pathlib import Path

DEFAULTS = {
    'api_port': 8765,

    # Topics
    'camera_topic': '/realsense/color/image_raw',
    'odom_topic': '/odometry/filtered',
    'map_topic': '/map',
    'candidate_topic': '/traction/nav2/candidate',
    # Text of QR codes, published by the computer_vision package.
    'qr_topic': '/qr_text',

    # How old sensor data may be before a task stops.
    'camera_max_age_s': 2.0,
    'sensor_max_age_s': 2.0,
    # Under load a camera frame can be late; allow short gaps.
    'camera_grace_s': 3.0,
    # Time to read the zone QR code after arriving.
    'zone_qr_timeout_s': 3.0,

    # Model timing
    'prediction_max_age_s': 6.0,
    'model_timeout_s': 5.0,
    'model_period_s': 1.0,
    # LightNav-0 is trained on frames 0.25 s apart (video_fps 4 in its eval_config).
    'frame_period_s': 0.25,
    # LightNav sometimes predicts stop for a single frame; require a run of them.
    'stop_confirmations': 3,

    # Visual tasks: replace a subgoal with a newer prediction after this long.
    'subgoal_refresh_s': 3.0,
    # After this many predictions the robot cannot follow, back up.
    'escape_after_unreachable': 2,
    'backup_distance_m': 0.3,
    'backup_speed': 0.15,

    # Model servers
    'lightnav_url': '',
    'foresight_service': '/legged_deployment/foresight_planner',
    'nav2_lifecycle_manager': '/lifecycle_manager_traction_navigation',

    # Subgoal limits
    'max_subgoal_distance_m': 2.0,
    'min_subgoal_distance_m': 0.12,
    'unknown_gap_m': 0.15,

    # Speed limits of the command gate
    'max_linear_speed': 0.19,
    'max_angular_speed': 2.4912,
    # Nav2 uses ROS (+angular.z turns left). The traction stack turns right
    # for +angular.z, on the robot and in Gazebo, so the gate flips the sign.
    'traction_turn_sign': -1.0,

    # Reversing, the tail aims at the goal line this far ahead of itself.
    'reverse_lookahead_m': 0.6,

    # Robot body, used for the footprint
    'num_modules': 3,
    'module_half_length': 0.23,
    'module_half_width': 0.16,
    # The arm camera sits 5 cm behind the front edge and its near clip is 0.28 m.
    # Padding the footprint ahead of it keeps walls far enough away to be seen.
    'camera_standoff_m': 0.25,

    'record_directory': str(Path.home() / '.local/state/reseq/traction'),
}
