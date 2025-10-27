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

    # TODO: with jinja we should be able to create use as a xacro file a compiled version that includes gazebo and other stuff
    # so, instead of picking the xacro file from the reseq_arm_mk2 we should compile xacro files and place them at compile
    # time in the share/urdf folder of this package `reseq_sim`
    xacro_file = get_package_share_directory('reseq_arm_mk2') + '/urdf/reseq_arm_mk2.xacro'
    robot_description = xacro.process_file(
        xacro_file,
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

    default_world = os.path.join(
        get_package_share_directory(package_name), 'worlds', 'empty.world'
    )

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
        get_package_share_directory('reseq_ros2'),
        'config',
        'gz_bridge.yaml',  # TODO: move gazebo_bridge node in this package
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

    return LaunchDescription(
        [
            # ExecuteProcess(
            #     cmd=['ign', 'gazebo', '-v', '4', 'libgazebo_ros_init.so', 'empty.sdf'],
            #     output='screen'),
            world_arg,
            rsp,
            gazebo,
            spawn_entity,
            ros_gz_bridge,
        ]
    )
