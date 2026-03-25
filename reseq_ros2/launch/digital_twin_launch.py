import os
import subprocess

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from reseq_ros2.utils.launch_utils import config_path, default_filename, parse_config

share_folder = get_package_share_directory('reseq_ros2')
description_share_folder = get_package_share_directory('reseq_description')


# launch_setup is used through an OpaqueFunction because it is the only way to manipulate a
# command line argument directly in the launch file
def launch_setup(context, *args, **kwargs):
    version = LaunchConfiguration('version').perform(context)
    # Get config path from command line, otherwise use the default path
    config_filename = LaunchConfiguration('config_file').perform(context)
    external_log_level = LaunchConfiguration('external_log_level').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)

    arm_arg = LaunchConfiguration('arm').perform(context=context)
    arm = True if arm_arg == 'true' else False  # bool version of arm_arg

    # Parse the config file
    config = parse_config(f'{config_path}/{version}/{config_filename}')
    launch_config = []

    description_share = get_package_share_directory('reseq_description')
    xacro_file = description_share_folder + f'/description/{version}/reseq.urdf.xacro'
    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'version': version,
            'config_path': f'{config_path}/{version}/{config_filename}',
            'sim_mode': sim_mode,
        },
    ).toxml()

    if version == 'mk2':
        robot_controllers = os.path.join(
            get_package_share_directory('reseq_ros2'),
            'config',
            'reseq_controllers_mk2.yaml',
        )
    else:
        generate_configs = subprocess.run(
            [
                'python3',
                os.path.join(description_share, 'scripts', 'generate_configs.py'),
                config_filename,
                '--version',
                version,
            ]
            + (['--use_sim_time'] if use_sim_time == 'true' else [])
            + (['--no_arm_controllers'] if not arm else []),
            check=True,
            capture_output=True,
            text=True,
        )

        if generate_configs.stdout:
            print(generate_configs.stdout)
        if generate_configs.stderr:
            print(generate_configs.stderr)

        robot_controllers = os.path.join(
            description_share, 'config', 'temp', 'reseq_controllers.yaml'
        )
    body_spawners = []
    arm_spawners = []
    if sim_mode == 'false':
        control_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                robot_controllers,
            ],
            output='both',
            arguments=['--ros-args', '--log-level', external_log_level],
        )
        launch_config.append(control_node)

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--ros-args',
            '--log-level',
            external_log_level,
        ],
    )
    body_spawners.append(joint_state_broadcaster_spawner)

    num_modules = config.get('num_modules', 0)
    for i in range(num_modules):
        module_controller = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                f'diff_controller{i + 1}',
                '--controller-manager',
                '/controller_manager',
                '--ros-args',
                '--log-level',
                external_log_level,
            ],
        )
        body_spawners.append(module_controller)

    if arm:
        mk2_arm_controller = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'mk2_arm_controller',
                '--controller-manager',
                '/controller_manager',
            ],
        )

        arm_spawners.append(mk2_arm_controller)

    if sim_mode == 'false':
        launch_config.append(
            RegisterEventHandler(
                OnProcessStart(
                    target_action=control_node,
                    on_start=[TimerAction(period=15.0, actions=[joint_state_broadcaster_spawner])],
                )
            )
        )
        if body_spawners[1:]:
            launch_config.append(
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=joint_state_broadcaster_spawner,
                        on_exit=[
                            TimerAction(
                                period=3.0,
                                actions=body_spawners[1:] + arm_spawners,
                            )
                        ],
                    )
                )
            )
    else:
        launch_config.extend(body_spawners + arm_spawners)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description}
        ],  # add other parameters here if required
        arguments=['--ros-args', '--log-level', external_log_level],
    )
    launch_config.append(robot_state_publisher_node)

    return launch_config


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('version', default_value='mk1', choices=['mk1', 'mk2']),
            DeclareLaunchArgument('config_file', default_value=default_filename),
            DeclareLaunchArgument(
                'arm',
                default_value='true',
                choices=['true', 'false'],
                description="Set to false if you don't want to use the arm",
            ),
            DeclareLaunchArgument('log_level', default_value='info'),
            DeclareLaunchArgument('external_log_level', default_value='warn'),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description=(
                    "set use_sim_time to 'true' if you are using gazebo."
                    ' In general this parameter is not set from this launch'
                    ' but instead is passed by other launch files that use this launch file.'
                    ' Setting this arg to true sets use_sim_time on all nodes launched here.'
                ),
            ),
            DeclareLaunchArgument('sim_mode', default_value='false'),
            OpaqueFunction(function=launch_setup),
        ]
    )
