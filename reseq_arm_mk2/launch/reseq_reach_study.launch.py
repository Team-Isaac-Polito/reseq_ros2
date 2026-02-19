from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Find package shares
    reseq_arm_mk2_share = FindPackageShare('reseq_arm_mk2')
    reach_ros_share = FindPackageShare('reach_ros')

    # Paths to reseq_arm_mk2 files
    robot_description_file = PathJoinSubstitution([reseq_arm_mk2_share, 'urdf', 'arm.urdf.xacro'])
    robot_description_semantic_file = PathJoinSubstitution(
        [reseq_arm_mk2_share, 'config', 'reseq_arm_mk2.srdf']
    )
    kinematics_file = PathJoinSubstitution([reseq_arm_mk2_share, 'config', 'kinematics.yaml'])
    reach_study_config_file = PathJoinSubstitution(
        [reseq_arm_mk2_share, 'config', 'reach_study.yaml']
    )
    reach_study_rviz_config = PathJoinSubstitution(
        [reseq_arm_mk2_share, 'viz', 'reseq_reach_study.rviz']
    )

    # Setup launch
    setup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([reach_ros_share, 'launch', 'setup.launch.py'])
        ),
        launch_arguments={
            'robot_description_file': robot_description_file,
            'robot_description_semantic_file': robot_description_semantic_file,
            'rviz_config': reach_study_rviz_config,
            'use_rviz': 'True',
        }.items(),
    )

    # Start launch
    start_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([reach_ros_share, 'launch', 'start.launch.py'])
        ),
        launch_arguments={
            'robot_description_file': robot_description_file,
            'robot_description_semantic_file': robot_description_semantic_file,
            'robot_description_kinematics_file': kinematics_file,
            # 'robot_description_joint_limits_file': joint_limits_file,
            'config_file': reach_study_config_file,
            'config_name': 'reseq_reach_study_pickik_run',
            'results_dir': './reach_results',
        }.items(),
    )

    return LaunchDescription(
        [
            setup_launch,
            start_launch,
        ]
    )
