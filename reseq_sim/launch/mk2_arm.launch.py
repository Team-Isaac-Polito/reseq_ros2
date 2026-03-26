import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'reseq_sim'
    description_share = get_package_share_directory('reseq_description')
    robot_config_file = os.path.join(description_share, 'config', 'mk2', 'reseq_mk2_vcan.yaml')

    generate_configs = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(description_share, 'scripts/generate_configs.py'),
            'reseq_mk2_vcan.yaml',
            '--use_sim_time',
            '--no_body_controllers',
            '--version',
            'mk2',
        ],
        name='generate_configs',
        output='screen',
    )

    # We must pass 'sim_mode='true' to xacro
    xacro_file = get_package_share_directory('reseq_arm_mk2') + '/urdf/arm.urdf.xacro'

    # Path to controllers config for the Gazebo plugin
    controllers_config_file = os.path.join(
        get_package_share_directory('reseq_description'),
        'config',
        'temp',
        'reseq_controllers.yaml',
    )
    print(f'Using controllers config file: {controllers_config_file}')

    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'sim_mode': 'true',
            'version': 'mk2',
            'config_path': robot_config_file,
            'controllers_config_file': controllers_config_file,
        },
    ).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
            }
        ],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name=package_name),
                    'launch',
                    'gazebo_launch.py',
                )
            ]
        )
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
    )

    joint_group_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_group_velocity_controller',
            '--controller-manager',
            '/controller_manager',
        ],
    )

    # Launch the direct Cartesian controller that publishes joint velocities
    # to the simulated arm controller. This keeps the Gazebo control path
    # local and avoids an extra trajectory interpolation layer.
    cartesian_arm_controller_node = Node(
        package='reseq_arm_mk2',
        executable='cartesian_arm_controller',
        name='cartesian_arm_controller',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
            {'chain_tip': 'tool0'},
            {'command_mode': 'velocity'},
            {'command_frame': 'arm_base_link'},
            {'max_cartesian_vel': 0.6},
            {'max_joint_vel': 1.0},
        ],
        output='screen',
    )

    # these nodes must all start after tehe configuration generation
    # hence the name `second_step`
    second_step = [
        rsp,
        gazebo_launch,
        TimerAction(period=15.0, actions=[joint_state_broadcaster_spawner]),
        cartesian_arm_controller_node,
    ]

    launch_description = LaunchDescription([generate_configs])
    launch_description.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=generate_configs,
                on_exit=second_step,
            )
        )
    )
    launch_description.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_group_velocity_controller_spawner],
            )
        )
    )
    return launch_description
