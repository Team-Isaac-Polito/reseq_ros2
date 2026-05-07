"""Launch SLAM Toolbox for 2D mapping with the RPLIDAR A2M8.

Uses online async mode by default. Pass ``slam_mode:=localization`` to
localise on an existing map instead of building a new one.

In mapping mode, also launches the ply_saver node which accumulates
colored PointCloud2 frames from the RGBD camera into reseq_map_3d.ply.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    reseq_share = get_package_share_directory('reseq_ros2')
    default_params = os.path.join(reseq_share, 'config', 'slam_toolbox.yaml')

    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode',
        default_value='mapping',
        choices=['mapping', 'localization'],
        description=(
            'SLAM mode: mapping (build new map) or localization (localise on an existing map)'
        ),
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

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'bond_timeout': 0.0,
                'node_names': ['slam_toolbox'],
            }
        ],
    )

    # ply_saver: accumulates colored PointCloud2 frames into a PLY map.
    ply_saver_node = Node(
        package='reseq_ros2',
        executable='ply_saver',
        name='ply_saver',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'pointcloud_topic': '/camera/depth/color/points',
                'save_path': '/ros2_ws/maps',
                'save_interval': 60.0,
                'voxel_size': 0.05,
                'frame_skip': 5,
                'max_range': 10.0,
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('slam_mode'), "' == 'mapping'"])
        ),
    )

    return LaunchDescription(
        [
            slam_mode_arg,
            params_arg,
            use_sim_time_arg,
            slam_node,
            lifecycle_manager,
            ply_saver_node,
        ]
    )
