"""Launch SLAM Toolbox for 2D mapping with the RPLIDAR A2M8.

Uses online async mode by default. Pass ``slam_mode:=localization`` to
localise on an existing map instead of building a new one.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    reseq_share = get_package_share_directory('reseq_ros2')
    default_params = os.path.join(reseq_share, 'config', 'slam_toolbox.yaml')

    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode',
        default_value='mapping',
        choices=['mapping', 'localization'],
        description='SLAM mode: mapping (build new map) or localization (localise on existing map)',
    )

    params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_params,
        description='Path to the SLAM Toolbox parameter file',
    )

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'mode': LaunchConfiguration('slam_mode'),
            },
        ],
    )

    return LaunchDescription(
        [
            slam_mode_arg,
            params_arg,
            use_sim_time_arg,
            slam_node,
        ]
    )
