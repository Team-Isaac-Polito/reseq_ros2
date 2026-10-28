"""Full MK2 simulation with one mux and one autonomous goal owner."""

from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def setup(context):
    core = Path(get_package_share_directory('reseq_ros2'))
    sim = Path(get_package_share_directory('reseq_sim'))
    use_sim = {'use_sim_time': True}
    world = LaunchConfiguration('world').perform(context)
    lightnav_url = LaunchConfiguration('lightnav_url').perform(context)
    port = int(LaunchConfiguration('api_port').perform(context))
    body_config = 'reseq_mk2_vcan.yaml'
    resources = [
        str(Path(get_package_share_directory('reseq_arm_mk2')).parent),
        str(Path(get_package_share_directory('reseq_description')).parent),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]
    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(resources)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(core / 'launch/reseq_launch.py')),
            launch_arguments={
                'version': 'mk2',
                'config_file': body_config,
                'arm': 'true',
                'use_sim_time': 'true',
                'sim_mode': 'true',
                'sensors': 'false',
                'slam': 'false',
                'autonomy': 'false',
                'app': 'true',
                'launch_yaw_controllers': 'true',
                'launch_pitch_controllers': 'true',
                'passive_yaw': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim / 'launch/gazebo_launch.py')),
            launch_arguments={
                'world': world,
                'spawn_x': LaunchConfiguration('spawn_x'),
                'spawn_y': LaunchConfiguration('spawn_y'),
                'spawn_z': '0.2',
                'num_modules': '3',
            }.items(),
        ),
    ]
    for sensor in ('up45', 'front', 'down45', 'down'):
        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'gazebo_tof_{sensor}_frame',
                arguments=[
                    '--frame-id',
                    f'tof_{sensor}_link',
                    '--child-frame-id',
                    f'reseq/base_link/tof_{sensor}',
                ],
                parameters=[use_sim],
            )
        )
    slam_params = ParameterFile(str(core / 'config/slam_toolbox.yaml'), allow_substs=True)
    actions += [
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, use_sim, {'use_lifecycle_manager': True}],
            remappings=[('/map', '/slam_map'), ('/map_metadata', '/slam_map_metadata')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_traction_slam',
            output='screen',
            parameters=[use_sim, {'autostart': True, 'node_names': ['slam_toolbox']}],
        ),
        Node(
            package='reseq_ros2',
            executable='map_republisher',
            name='map_republisher',
            parameters=[
                use_sim,
                {'input_topic': '/slam_map', 'output_topic': '/map', 'publish_rate': 1.0},
            ],
            output='screen',
        ),
    ]
    parameters = ParameterFile(str(core / 'config/nav2_params.yaml'), allow_substs=True)
    common = [parameters, use_sim]
    # Explicit topic routing keeps Nav2's raw command away from the existing mux.
    descriptions = [
        (
            'nav2_controller',
            'controller_server',
            [('cmd_vel', '/traction/nav2/raw'), ('odom', '/diff_controller1/odom')],
            {},
        ),
        ('nav2_planner', 'planner_server', [], {}),
        ('nav2_behaviors', 'behavior_server', [('cmd_vel', '/traction/nav2/raw')], {}),
        ('nav2_bt_navigator', 'bt_navigator', [], {}),
        (
            'nav2_velocity_smoother',
            'velocity_smoother',
            [('cmd_vel', '/traction/nav2/raw'), ('cmd_vel_smoothed', '/traction/nav2/smoothed')],
            {},
        ),
        (
            'nav2_collision_monitor',
            'collision_monitor',
            [],
            {
                'cmd_vel_in_topic': '/traction/nav2/smoothed',
                'cmd_vel_out_topic': '/traction/nav2/candidate',
            },
        ),
    ]
    for package, executable, remappings, overrides in descriptions:
        actions.append(
            Node(
                package=package,
                executable=executable,
                name=executable,
                output='screen',
                parameters=[*common, overrides],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'), *remappings],
            )
        )
    actions += [
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_traction_navigation',
            output='screen',
            parameters=[
                use_sim,
                {'autostart': True, 'node_names': [item[1] for item in descriptions]},
            ],
        ),
        Node(
            package='reseq_traction_autonomy',
            executable='traction_manager',
            name='traction_manager',
            prefix=LaunchConfiguration('manager_python'),
            output='screen',
            parameters=[use_sim, {'api_port': port, 'lightnav_url': lightnav_url}],
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('world', default_value='simple_course.world'),
            DeclareLaunchArgument('spawn_x', default_value='0.0'),
            DeclareLaunchArgument('spawn_y', default_value='0.0'),
            DeclareLaunchArgument('api_port', default_value='8765'),
            DeclareLaunchArgument('manager_python', default_value='/usr/bin/python3'),
            DeclareLaunchArgument('lightnav_url', default_value=''),
            OpaqueFunction(function=setup),
        ]
    )
