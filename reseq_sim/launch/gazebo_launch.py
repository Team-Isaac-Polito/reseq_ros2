import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'reseq_sim'

    ####################
    # LAUNCH ARGUMENTS #
    ####################

    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.world', description='World to load in the gazebo simulation'
    )

    bridge_file_arg = DeclareLaunchArgument(
        'bridge_file',
        default_value='gz_bridge.yaml',
        description='File containing bridge topics between gazebo and ros2',
    )

    ###################################
    # START GAZEBO AND SPAWN ENTITIES #
    ###################################

    # Launch gazebo with the world specified in `world_path`
    world_path = PathJoinSubstitution(
        [get_package_share_directory(package_name), 'worlds', LaunchConfiguration('world')]
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': ['-r -v4 ', world_path], 'on_exit_shutdown': 'true'}.items(),
    )

    # the `create` node is in charge of spawing the robot in gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'reseq', '-topic', '/robot_description'],
        output='screen',
    )

    ##########
    # BRIDGE #
    ##########

    bridge_file_path = PathJoinSubstitution(
        [get_package_share_directory(package_name), 'config', LaunchConfiguration('bridge_file')]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_file_path}],
    )

    return LaunchDescription(
        [
            world_arg,
            bridge_file_arg,
            gazebo,
            spawn_entity,
            ros_gz_bridge,
        ]
    )
