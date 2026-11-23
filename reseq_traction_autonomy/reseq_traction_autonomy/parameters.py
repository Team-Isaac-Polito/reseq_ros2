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
    # A stop is ignored when the map shows a wall this close ahead of the front module.
    # The camera then sees only the wall. Low objects such as a lying person are below
    # the lidar, so they never count as a wall.
    'stop_wall_distance_m': 0.6,

    # Visual tasks: replace a subgoal with a newer prediction after this long.
    'subgoal_refresh_s': 3.0,
    # A turn arc is a short manoeuvre; let it finish unless it takes longer than this.
    'arc_refresh_s': 12.0,
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

    # Rese.Q cannot turn on the spot. When the model asks for that, drive an arc
    # that turns the same way: forward if there is room, else in reverse.
    'turn_arcs': True,
    # Turns smaller than this are ignored.
    'min_turn_rad': 0.15,
    'turn_arc_length_m': 0.8,
    # Larger turns are done in several arcs, one per prediction.
    'turn_arc_max_rad': 0.8,

    # Exploration (visual tasks started with explore): frontiers are the edges of the
    # mapped area. Shorter ones are usually gaps between lidar rays.
    'frontier_min_size_m': 0.5,
    # Score = distance - weight * size, so a large frontier is worth a longer drive.
    'frontier_size_weight': 0.5,
    # Frontiers closer than this to the robot are skipped: the map there is still filling in.
    'frontier_min_distance_m': 1.0,
    # Frontiers this close to one tried before are skipped.
    'frontier_tried_radius_m': 1.0,
    # How far back from a frontier the robot stands, nearest first.
    'frontier_standoffs_m': [0.6, 0.9, 1.2, 1.5],
    # A frontier goal is a longer drive; give it this long before a newer prediction wins.
    'frontier_refresh_s': 20.0,
    # Exploration without a model: give up on a frontier after this long,
    # and finish once no frontier has been left for this long.
    'explore_goal_timeout_s': 60.0,
    'explore_done_after_s': 10.0,

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
    # The other modules follow the head's path, so driving forward Nav2 checks only the
    # head, and goals are checked with the head only. Reversing always uses the whole
    # body. False uses the hull of the whole bent body for everything, as before.
    'footprint_head_only': True,
    'module_half_length': 0.23,
    'module_half_width': 0.16,
    # The arm camera sits 5 cm behind the front edge and its near clip is 0.28 m.
    # Padding the footprint ahead of it keeps walls far enough away to be seen.
    'camera_standoff_m': 0.25,

    'record_directory': str(Path.home() / '.local/state/reseq/traction'),
}
