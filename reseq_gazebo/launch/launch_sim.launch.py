import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled

    package_name = 'reseq_gazebo'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(
                package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(
        package_name), 'config', 'gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'reseq_gazebo'],
                        output='screen')
    
    
    diff_drive_1_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont_17"],
    )
    
    diff_drive_2_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont_18"],
    )
    
    diff_drive_3_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont_19"],
    )
    
    diff_drive_4_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont_20"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_1_spawner,
        diff_drive_2_spawner,
        diff_drive_3_spawner,
        diff_drive_4_spawner,
        joint_broad_spawner
    ])
