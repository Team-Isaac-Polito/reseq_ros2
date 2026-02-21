import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_name = 'reseq_sim'
    description_share = get_package_share_directory('reseq_description')

    generate_configs = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(description_share, 'scripts/generate_configs.py'),
            'reseq_mk2_can.yaml',
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

    mk2_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mk2_arm_controller',
            '--controller-manager',
            '/controller_manager',
        ],
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

    # Load all MoveIt configuration files
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='reseq_arm_mk2',
            package_name='reseq_arm_mk2',
        )
        .robot_description(
            file_path='urdf/reseq_arm_mk2.xacro',
            # We must pass the same arguments as the robot_state_publisher
            mappings={
                'sim_mode': 'true',
                'controllers_config_file': controllers_config_file,
            },
        )
        .robot_description_semantic(file_path='config/reseq_arm_mk2.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        # .joint_limits(file_path='config/joint_limits.yaml')
        .to_moveit_configs()
    )

    # Get the path to the servo config file
    servo_config_file = os.path.join(
        get_package_share_directory('reseq_arm_mk2'), 'config', 'servo_config.yaml'
    )

    # Start move_group node (required for planning and IK)
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ],
    )

    servo_node_exec = "servo_node_main" if os.environ.get('ROS_DISTRO') == 'humble' else "servo_node"

    # Define the servo node, passing ALL required configs
    servo_node = Node(
        package='moveit_servo',
        executable=servo_node_exec,
        name='moveit_servo_node',
        parameters=[
            moveit_config.to_dict(),  # All MoveIt config
            servo_config_file,  # Servo config file (MUST BE LAST)
            {'use_sim_time': True},  # Don't forget this for Gazebo
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    moveit_controller_node = Node(
        package='reseq_arm_mk2',
        executable='moveit_controller.py',
        name='moveit_controller',
        parameters=[
            {
                'arm_module_address': 0,
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    joint_group_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_group_velocity_controller',
            '--controller-manager',
            '/controller_manager',
            '--inactive',
        ],
    )

    # these nodes must all start after tehe configuration generation
    # hence the name `second_step`
    second_step = [
        rsp,
        mk2_arm_controller_spawner,
        joint_group_velocity_controller_spawner,
        gazebo_launch,
        joint_state_broadcaster_spawner,
        move_group_node,
        servo_node,
        moveit_controller_node,
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
    return launch_description
