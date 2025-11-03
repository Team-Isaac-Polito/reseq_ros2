import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

from reseq_ros2.utils.launch_utils import config_path, default_filename, parse_config


def launch_setup(context, *args, **kwargs):
    config_filename = LaunchConfiguration('config_file').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    use_webcam = LaunchConfiguration('use_webcam').perform(context)

    config = parse_config(f'{config_path}/{config_filename}')
    launch_config = []

    if use_webcam == 'false':
        # Hardware specific nodes (disabled for usb webcam testing)

        # Communication node
        communication_node = Node(
            package='reseq_ros2',
            executable='communication',
            name='communication',
            parameters=[
                ParameterFile(f'{config_path}/{config["can_config"]}'),
            ],
            arguments=['--ros-args', '--log-level', log_level],
        )
        launch_config.append(communication_node)

        # Agevar node
        agevar_node = Node(
            package='reseq_ros2',
            executable='agevar',
            name='agevar',
            parameters=[
                ParameterFile(f'{config_path}/{config["agevar_consts"]}'),
            ],
            arguments=['--ros-args', '--log-level', log_level],
        )
        launch_config.append(agevar_node)

        # Pivot controller node
        pivot_controller_node = Node(
            package='reseq_ros2',
            executable='pivot_controller',
            name='pivot_controller',
            arguments=['--ros-args', '--log-level', log_level],
        )
        launch_config.append(pivot_controller_node)

        # Scaler node
        scaler_node = Node(
            package='reseq_ros2',
            executable='scaler',
            name='scaler',
            parameters=[
                ParameterFile(f'{config_path}/{config["scaler_consts"]}'),
            ],
            arguments=['--ros-args', '--log-level', log_level],
        )
        launch_config.append(scaler_node)

        # joint_publisher
        joint_publisher_node = Node(
            package='reseq_ros2',
            executable='joint_publisher',
            name='joint_publisher',
            parameters=[
                ParameterFile(f'{config_path}/{config["joint_pub_consts"]}'),
            ],
            arguments=['--ros-args', '--log-level', log_level],
        )
        launch_config.append(joint_publisher_node)

        # ros2_control
        ros2_control_launch_file = os.path.join(
            get_package_share_directory('reseq_ros2'), 'launch', 'ros2_control_launch.py'
        )
        launch_config.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ros2_control_launch_file),
                launch_arguments={
                    'config_file': config_filename,
                    'log_level': log_level,
                }.items(),
            )
        )

    return launch_config


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('config_file', default_value=default_filename),
            DeclareLaunchArgument('log_level', default_value='info'),
            DeclareLaunchArgument('use_webcam', default_value='false'),
            OpaqueFunction(function=launch_setup),
        ]
    )
