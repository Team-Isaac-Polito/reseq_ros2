import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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
    use_sim_time_arg = LaunchConfiguration('use_sim_time').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)

    arm_arg = LaunchConfiguration('arm').perform(context=context)
    arm = True if arm_arg == 'true' else False  # bool version of arm_arg

    # Parse the config file
    config = parse_config(f'{config_path}/{version}/{config_filename}')
    xacro_file = description_share_folder + f'/description/{version}/reseq.urdf.xacro'
    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'version': version,
            'config_path': f'{config_path}/{version}/{config_filename}',
            'arm': arm_arg,
            'sim_mode': sim_mode,
        },
    ).toxml()
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time_arg == 'true',
            }
        ],  # add other parameters here if required
        arguments=['--ros-args', '--log-level', external_log_level],
    )

    launch_config = [robot_state_publisher_node]

    robot_controllers = f'{config_path}/reseq_controllers.yaml'
    if sim_mode == 'false':
        control_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_controllers],
            output='both',
            remappings=[
                ('~/robot_description', '/robot_description'),
            ],
            arguments=['--ros-args', '--log-level', external_log_level],
        )
        launch_config.append(control_node)

    spawners = []
    spawners.append(
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager',
                '/controller_manager',
                '--switch-timeout',
                '30.0',
                '--ros-args',
                '--log-level',
                external_log_level,
            ],
        )
    )

    num_modules = config.get('num_modules', 0)
    for i in range(num_modules):
        spawners.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    f'diff_controller{i + 1}',
                    '--controller-manager',
                    '/controller_manager',
                    '--switch-timeout',
                    '30.0',
                    '--ros-args',
                    '--log-level',
                    external_log_level,
                ],
            )
        )

    # Spawn yaw joint controllers (ForwardCommandController — sim only)
    if use_sim_time_arg == 'true':
        for i in range(num_modules - 1):
            spawners.append(
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        f'yaw_controller{i + 2}',
                        '--controller-manager',
                        '/controller_manager',
                        '--switch-timeout',
                        '30.0',
                        '--ros-args',
                        '--log-level',
                        external_log_level,
                    ],
                )
            )

    # Spawn IMU sensor broadcasters (reads hardware state interfaces from either
    # GazeboSimSystem or ReseqHardware and publishes sensor_msgs/Imu to /{name}/imu)
    for i in range(num_modules):
        spawners.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    f'imu{i + 1}_broadcaster',
                    '--controller-manager',
                    '/controller_manager',
                    '--switch-timeout',
                    '30.0',
                    '--ros-args',
                    '--log-level',
                    external_log_level,
                ],
            )
        )

    if arm:
        spawners.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'mk2_arm_controller',
                    '--controller-manager',
                    '/controller_manager',
                    '--switch-timeout',
                    '30.0',
                ],
            )
        )
        spawners.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_group_velocity_controller',
                    '--controller-manager',
                    '/controller_manager',
                    '--inactive',
                    '--switch-timeout',
                    '30.0',
                ],
            )
        )

    if spawners:
        launch_config.append(spawners[0])
        for previous_spawner, current_spawner in zip(spawners, spawners[1:]):
            launch_config.append(
                RegisterEventHandler(
                    OnProcessExit(target_action=previous_spawner, on_exit=[current_spawner])
                )
            )

    # EKF node: fuse wheel odometry + IMU to reduce angular drift from wheel slip
    ekf_config = os.path.join(share_folder, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time_arg == 'true'}],
        arguments=['--ros-args', '--log-level', external_log_level],
    )
    launch_config.append(ekf_node)

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
                    "set use_sim_time to 'true' if you are using gazebo. "
                    'In general this parameter is not set from this launch '
                    'but instead is passed by other launch files that use '
                    "this launch file. Setting this arg to 'true' sets the "
                    'use_sim_time parameter of all nodes launched in this '
                    'file to True.'
                ),
            ),
            DeclareLaunchArgument('sim_mode', default_value='false'),
            OpaqueFunction(function=launch_setup),
        ]
    )
