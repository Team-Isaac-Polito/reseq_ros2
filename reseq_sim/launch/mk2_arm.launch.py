import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'reseq_sim'

    # Get the share directory for this package
    reseq_sim_share_dir = get_package_share_directory(package_name)

    # We must pass 'sim_mode'='true' to xacro
    xacro_file = get_package_share_directory('reseq_arm_mk2') + '/urdf/reseq_arm_mk2.xacro'

    # Path to controllers config for the Gazebo plugin
    controllers_config_file = os.path.join(
        get_package_share_directory('reseq_ros2'),
        'config',
        'reseq_controllers.yaml',
    )

    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'sim_mode': 'true',
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

    default_world = os.path.join(reseq_sim_share_dir, 'worlds', 'empty.world')

    print(f'Default world path: {default_world}')

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world', default_value=default_world, description='world_to_load'
    )

    # Include the gazebo launch file provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items(),
    )

    # Include the executable can spawn the robot model in the simulation
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'reseq', '-topic', '/robot_description'],
        output='screen',
    )

    bridge_params = os.path.join(
        reseq_sim_share_dir,
        'config',
        'gz_bridge.yaml',
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
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

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager',
        ],
    )

    return LaunchDescription(
        [
            world_arg,
            rsp,
            gazebo,
            spawn_entity,
            ros_gz_bridge,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
        ]
    )
