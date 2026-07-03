"""Launch SLAM Toolbox for 2D mapping with the RPLIDAR A2M8.

Uses online async mode by default. Pass ``slam_mode:=localization`` to
localise on an existing map instead of building a new one.

In mapping mode, also launches:
- ply_saver node which accumulates colored PointCloud2 frames from the RGBD camera into reseq_map_3d.ply
- geotiff_node which periodically saves the 2D occupancy grid map as GeoTIFF (every 60s and on shutdown)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
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
    ply_save_path_arg = DeclareLaunchArgument(
        'ply_save_path',
        default_value=os.environ.get('RESEQ_PLY_SAVE_PATH', '/ros2_ws/maps'),
        description='Directory where ply_saver writes the 3D map',
    )

    geotiff_save_path_arg = DeclareLaunchArgument(
        'geotiff_save_path',
        default_value=os.environ.get('RESEQ_GEOTIFF_SAVE_PATH', '/ros2_ws/maps'),
        description='Directory where geotiff_node writes the 2D GeoTIFF map',
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=[
            ('/map', '/slam_map'),
            ('/map_metadata', '/slam_map_metadata'),
        ],
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'mode': LaunchConfiguration('slam_mode'),
            },
        ],
    )

    map_republisher_node = Node(
        package='reseq_ros2',
        executable='map_republisher',
        name='map_republisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': '/slam_map',
                'output_topic': '/map',
                'metadata_topic': '/map_metadata',
                'publish_rate': 1.0,
            }
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
                'pointcloud_topic': '/realsense/depth/color/points',
                'save_path': LaunchConfiguration('ply_save_path'),
                'save_interval': 60.0,
                'voxel_size': 0.05,
                'frame_skip': 5,
                'max_range': 10.0,
                'scan_topic': '/scan',
                'scan_frame_skip': 2,
                'scan_max_range': 12.0,
                'scan_color_rgb': [255, 210, 0],
                'wait_for_map': True,
                'map_timeout_sec': 60.0,
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('slam_mode'), "' == 'mapping'"])
        ),
    )

    # geotiff_node: periodically saves the 2D occupancy grid map as GeoTIFF
    # Saves every 60 seconds and on shutdown via syscommand topic
    geotiff_node = Node(
        package='is_geotiff',
        executable='geotiff_node',
        name='geotiff_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_file_path': LaunchConfiguration('geotiff_save_path'),
                'map_file_base_name': 'RoboCup2026-ISAAC-P2',
                'geotiff_save_period': 60.0,
                'draw_background_checkerboard': True,
                'draw_free_space_grid': True,
                'use_map_topic': True,
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('slam_mode'), "' == 'mapping'"])
        ),
    )

    # Register shutdown handler to trigger geotiff save on shutdown
    shutdown_geotiff = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                Node(
                    package='reseq_ros2',
                    executable='geotiff_shutdown_saver',
                    name='geotiff_shutdown_saver',
                    output='screen',
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                )
            ]
        )
    )

    return LaunchDescription(
        [
            slam_mode_arg,
            params_arg,
            use_sim_time_arg,
            ply_save_path_arg,
            geotiff_save_path_arg,
            slam_node,
            map_republisher_node,
            lifecycle_manager,
            ply_saver_node,
            geotiff_node,
            shutdown_geotiff,
        ]
    )
