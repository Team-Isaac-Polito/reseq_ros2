import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
 
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "reseq_ros2"

    #Include the standard launch file which contain the robot_state_publisher node
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(
                package_name), 'launch', 'reseq_launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()  
    )

    #Include the gazebo launch file provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            )

    #Include the executable can spawn the robot model in the simulation
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description', 
                                   '-entity', 'reseq_ros2'],
                        output='screen')               

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity
    ])
