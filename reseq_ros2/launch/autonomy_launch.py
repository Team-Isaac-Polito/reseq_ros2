import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def launch_setup(context, *args, **kwargs):
    nav2_share = get_package_share_directory('nav2_bringup')
    map_file = LaunchConfiguration('map_file').perform(context).strip()

    nav2_launch = GroupAction(
        actions=[
            SetRemap(src='odom', dst='/diff_controller1/odom'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_share, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': LaunchConfiguration('nav2_params_file'),
                    'autostart': LaunchConfiguration('autostart'),
                }.items(),
            ),
        ]
    )

    mux_parameters = [
        LaunchConfiguration('autonomy_params_file'),
        {'use_sim_time': LaunchConfiguration('use_sim_time')},
    ]
    if map_file:
        mux_parameters.append({'require_recent_map': False})

    mux_node = Node(
        package='reseq_ros2',
        executable='cmd_vel_mux',
        name='cmd_vel_mux',
        output='screen',
        parameters=mux_parameters,
    )

    explorer_node = Node(
        package='reseq_ros2',
        executable='autonomy_coordinator',
        name='autonomy_coordinator',
        output='screen',
        parameters=[
            LaunchConfiguration('autonomy_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    launch_actions = []
    if map_file:
        launch_actions.extend(
            [
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[
                        {
                            'use_sim_time': LaunchConfiguration('use_sim_time'),
                            'yaml_filename': LaunchConfiguration('map_file'),
                        }
                    ],
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_map',
                    output='screen',
                    parameters=[
                        {
                            'use_sim_time': LaunchConfiguration('use_sim_time'),
                            'autostart': True,
                            'node_names': ['map_server'],
                        }
                    ],
                ),
            ]
        )

    launch_actions.extend([nav2_launch, mux_node, explorer_node])
    return launch_actions


def generate_launch_description():
    reseq_share = get_package_share_directory('reseq_ros2')
    default_nav2_params = os.path.join(reseq_share, 'config', 'nav2_params.yaml')
    default_autonomy_params = os.path.join(reseq_share, 'config', 'autonomy_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')
    params_arg = DeclareLaunchArgument('nav2_params_file', default_value=default_nav2_params)
    autonomy_params_arg = DeclareLaunchArgument(
        'autonomy_params_file', default_value=default_autonomy_params
    )
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Optional static map yaml file for simulation/localization-free navigation',
    )
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='Robot spawn x position used to align the static map with odom',
    )
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Robot spawn y position used to align the static map with odom',
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            autostart_arg,
            params_arg,
            autonomy_params_arg,
            map_file_arg,
            spawn_x_arg,
            spawn_y_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
