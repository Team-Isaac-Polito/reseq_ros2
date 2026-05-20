import os
import subprocess

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from reseq_ros2.utils.launch_utils import config_path, default_filename, parse_config

share_folder = get_package_share_directory('reseq_ros2')
description_share_folder = get_package_share_directory('reseq_description')


def _spawner(name: str, external_log_level: str, *extra_args: str) -> Node:
    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            name,
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '60',
            '--service-call-timeout',
            '60',
            *extra_args,
            '--ros-args',
            '--log-level',
            external_log_level,
        ],
    )


def _append_spawner_chain(launch_config, start_action, spawners):
    if not spawners:
        return

    launch_config.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=start_action,
                on_exit=[spawners[0]],
            )
        )
    )

    for previous_spawner, next_spawner in zip(spawners, spawners[1:]):
        launch_config.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=previous_spawner,
                    on_exit=[next_spawner],
                )
            )
        )


# launch_setup is used through an OpaqueFunction because it is the only way to manipulate a
# command line argument directly in the launch file
def launch_setup(context, *args, **kwargs):
    version = LaunchConfiguration('version').perform(context)
    config_filename = LaunchConfiguration('config_file').perform(context)
    external_log_level = LaunchConfiguration('external_log_level').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    sim_branch_use_sim_time = 'true' if sim_mode == 'true' else use_sim_time
    use_moveit = LaunchConfiguration('use_moveit').perform(context).lower() == 'true'
    launch_yaw_controllers = (
        LaunchConfiguration('launch_yaw_controllers').perform(context).lower() == 'true'
    )
    arm_max_cartesian_vel = float(LaunchConfiguration('arm_max_cartesian_vel').perform(context))
    arm_max_angular_vel = float(LaunchConfiguration('arm_max_angular_vel').perform(context))
    arm_max_joint_vel = float(LaunchConfiguration('arm_max_joint_vel').perform(context))
    arm_robot_forward_rpy = [
        float(v)
        for v in LaunchConfiguration('arm_robot_forward_rpy')
        .perform(context)
        .replace(',', ' ')
        .split()
    ]

    arm_arg = LaunchConfiguration('arm').perform(context=context)
    arm = arm_arg == 'true'

    description_share = get_package_share_directory('reseq_description')
    generate_configs = subprocess.run(
        [
            'python3',
            os.path.join(description_share, 'scripts', 'generate_configs.py'),
            config_filename,
            '--version',
            version,
        ]
        + (['--use_sim_time'] if sim_branch_use_sim_time == 'true' else [])
        + (['--no_arm_controllers'] if not arm else []),
        check=True,
        capture_output=True,
        text=True,
    )
    if generate_configs.stdout:
        print(generate_configs.stdout)
    if generate_configs.stderr:
        print(generate_configs.stderr)

    config = parse_config(f'{config_path}/{version}/{config_filename}')
    robot_controllers = os.path.join(description_share, 'config', 'temp', 'reseq_controllers.yaml')

    xacro_file = os.path.join(description_share_folder, 'description', version, 'reseq.urdf.xacro')
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
                'use_sim_time': sim_branch_use_sim_time == 'true',
                'publish_frequency': 50.0,
            }
        ],
        arguments=['--ros-args', '--log-level', external_log_level],
    )

    launch_config = [robot_state_publisher_node]

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

    joint_state_spawner = _spawner('joint_state_broadcaster', external_log_level)
    body_spawners = []

    num_modules = config.get('num_modules', 0)
    for i in range(num_modules):
        body_spawners.append(_spawner(f'diff_controller{i + 1}', external_log_level))

    if sim_branch_use_sim_time == 'true' and launch_yaw_controllers:
        for i in range(num_modules - 1):
            body_spawners.append(_spawner(f'yaw_controller{i + 2}', external_log_level))

    arm_velocity_spawner = None
    if arm:
        arm_velocity_spawner = _spawner('joint_group_velocity_controller', external_log_level)

    for i in range(num_modules):
        body_spawners.append(_spawner(f'imu{i + 1}_broadcaster', external_log_level))

    controller_manager_ready = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            'until ros2 service type /controller_manager/list_controllers '
            '> /dev/null 2>&1; do sleep 1; done',
        ],
        output='screen',
    )

    if sim_mode == 'false':
        launch_config.append(
            RegisterEventHandler(
                OnProcessStart(target_action=control_node, on_start=[controller_manager_ready])
            )
        )
    else:
        launch_config.append(controller_manager_ready)

    launch_config.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=controller_manager_ready,
                on_exit=[joint_state_spawner],
            )
        )
    )
    _append_spawner_chain(launch_config, joint_state_spawner, body_spawners)
    if arm_velocity_spawner is not None:
        launch_config.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_spawner,
                    on_exit=[arm_velocity_spawner],
                )
            )
        )

    ekf_config = os.path.join(share_folder, 'config', 'ekf.yaml')
    launch_config.append(
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': sim_branch_use_sim_time == 'true'}],
            arguments=['--ros-args', '--log-level', external_log_level],
        )
    )

    if sim_mode == 'false' and arm:
        launch_config.append(
            Node(
                package='reseq_arm_mk2',
                executable='arm_state_bridge',
                name='arm_state_bridge',
                parameters=[
                    {
                        'source_topic': '/joint_states',
                        'output_mode': 'joint_state',
                        'output_topic': '/arm_joint_states',
                    }
                ],
                output='screen',
            )
        )

    if arm:
        arm_chain_tip = 'cameras_holder_link' if sim_mode == 'true' else 'tcp'
        arm_state_topic = '/joint_states' if sim_mode == 'true' else '/arm_joint_states'
        cartesian_arm_node = Node(
            package='reseq_arm_mk2',
            executable='cartesian_arm_controller.py',
            name='cartesian_arm_controller',
            parameters=[
                {
                    'robot_description': robot_description,
                    'use_sim_time': sim_branch_use_sim_time == 'true',
                    'state_topic': arm_state_topic,
                    'velocity_topic': '/mk2_arm_vel_scaled',
                    'chain_tip': arm_chain_tip,
                    'command_frame': 'arm_base_link',
                    'command_mode': 'velocity',
                    'max_cartesian_vel': arm_max_cartesian_vel,
                    'max_angular_vel': arm_max_angular_vel,
                    'max_joint_vel': arm_max_joint_vel,
                    'robot_forward_rpy': arm_robot_forward_rpy,
                    'deadzone': 0.02,
                    'trajectory_horizon_sec': 0.1,
                }
            ],
            output='screen',
        )
        if arm_velocity_spawner is not None:
            launch_config.append(
                RegisterEventHandler(
                    OnProcessExit(target_action=arm_velocity_spawner, on_exit=[cartesian_arm_node])
                )
            )
        else:
            launch_config.append(cartesian_arm_node)

    if sim_mode == 'false' and arm and use_moveit:
        launch_config.append(
            Node(
                package='reseq_arm_mk2',
                executable='coordinate_controller.py',
                name='coordinate_controller',
                parameters=[{'robot_description': robot_description}],
                output='screen',
            )
        )

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
                    'but instead is passed by other launch files that use this launch file. '
                    "Setting this arg to 'true' sets use_sim_time on all launched nodes."
                ),
            ),
            DeclareLaunchArgument('sim_mode', default_value='false'),
            DeclareLaunchArgument(
                'arm_max_cartesian_vel',
                default_value='0.4',
                description='Cartesian velocity scale for the arm controller',
            ),
            DeclareLaunchArgument(
                'arm_max_angular_vel',
                default_value='0.8',
                description='Angular velocity scale for arm rotation mode',
            ),
            DeclareLaunchArgument(
                'arm_max_joint_vel',
                default_value='0.8',
                description='Joint velocity clamp for the arm controller',
            ),
            DeclareLaunchArgument(
                'arm_robot_forward_rpy',
                default_value='0.0 0.0 0.0',
                description='Fixed robot-forward tool orientation RPY relative to arm_base_link',
            ),
            DeclareLaunchArgument(
                'use_moveit',
                default_value='false',
                description='Launch MoveIt-related arm tools',
            ),
            DeclareLaunchArgument(
                'launch_yaw_controllers',
                default_value='false',
                description='Launch yaw position controllers in simulation/hardware control stack',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
