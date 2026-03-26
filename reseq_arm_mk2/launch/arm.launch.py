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


def launch_setup(context, *args, **kwargs):
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    use_moveit = LaunchConfiguration('use_moveit').perform(context).lower() == 'true'

    description_share = get_package_share_directory('reseq_description')
    arm_share = get_package_share_directory('reseq_arm_mk2')

    config_file = 'reseq_mk2_vcan.yaml' if sim else 'reseq_mk2_can.yaml'
    robot_config_file = os.path.join(description_share, 'config', 'mk2', config_file)
    generate_configs = subprocess.run(
        [
            'python3',
            os.path.join(description_share, 'scripts', 'generate_configs.py'),
            config_file,
            '--version',
            'mk2',
            '--no_body_controllers',
        ]
        + (['--use_sim_time'] if sim else []),
        check=True,
        capture_output=True,
        text=True,
    )

    if generate_configs.stdout:
        print(generate_configs.stdout)
    if generate_configs.stderr:
        print(generate_configs.stderr)

    controllers_config_file = os.path.join(
        description_share, 'config', 'temp', 'reseq_controllers.yaml'
    )
    robot_description_file = os.path.join(arm_share, 'urdf', 'arm.urdf.xacro')
    robot_description = xacro.process_file(
        robot_description_file,
        mappings={
            'sim_mode': 'true' if sim else 'false',
            'version': 'mk2',
            'config_path': robot_config_file,
            'controllers_config_file': controllers_config_file,
        },
    ).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    cartesian_arm_controller_node = Node(
        package='reseq_arm_mk2',
        executable='cartesian_arm_controller',
        name='cartesian_arm_controller',
        parameters=[
            {
                'robot_description': robot_description,
                'state_topic': '/arm_joint_states',
                'chain_tip': 'tool0',
                'command_frame': 'arm_base_link',
                'command_mode': 'trajectory',
                'max_cartesian_vel': 0.6,
                'max_joint_vel': 1.0,
                'deadzone': 0.0,
                'trajectory_horizon_sec': 0.1,
                'joint_weights': [3.0, 2.5, 1.4, 1.0, 0.7, 0.7],
            }
        ],
        output='screen',
    )

    launch_entities = [
        robot_state_publisher_node,
        cartesian_arm_controller_node,
    ]

    if not sim:
        control_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controllers_config_file,
            ],
            output='screen',
        )

        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager',
                '/controller_manager',
                '--controller-manager-timeout',
                '60',
                '--service-call-timeout',
                '60',
            ],
        )

        controller_manager_ready = ExecuteProcess(
            cmd=[
                'bash',
                '-lc',
                (
                    'until ros2 service list --include-hidden-services '
                    '| grep -Fxq /controller_manager/list_controllers; '
                    'do sleep 1; done'
                ),
            ],
            output='screen',
        )

        arm_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'mk2_arm_controller',
                '--controller-manager',
                '/controller_manager',
                '--controller-manager-timeout',
                '60',
                '--service-call-timeout',
                '60',
            ],
        )

        launch_entities.extend(
            [
                control_node,
                RegisterEventHandler(
                    OnProcessStart(
                        target_action=control_node,
                        on_start=[controller_manager_ready],
                    )
                ),
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=controller_manager_ready,
                        on_exit=[joint_state_broadcaster_spawner],
                    )
                ),
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=joint_state_broadcaster_spawner,
                        on_exit=[arm_controller_spawner],
                    )
                ),
            ]
        )

    if use_moveit:
        launch_entities.append(
            Node(
                package='reseq_arm_mk2',
                executable='coordinate_controller',
                name='coordinate_controller',
                parameters=[{'robot_description': robot_description}],
                output='screen',
            )
        )

    return launch_entities


def generate_launch_description():

    # ── Declare arguments ──────────────────────────────────────────────────
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Use Gazebo simulation controllers',
    )
    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit',
        default_value='true',
        description='Launch MoveIt (for IK / RViz visualisation)',
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logger level (debug|info|warn|error)',
    )

    return LaunchDescription(
        [
            sim_arg,
            use_moveit_arg,
            log_level_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
