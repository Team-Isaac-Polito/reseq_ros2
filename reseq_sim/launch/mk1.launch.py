import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    mk1_package_name = 'reseq_ros2'
    package_name = 'reseq_sim'

    # Include the standard launch file which contain the robot_state_publisher node
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(mk1_package_name), 'launch', 'reseq_launch.py'
                )
            ]
        ),
        launch_arguments={
            'version': 'mk1',
            'use_sim_time': 'true',
            'use_ros2_control': 'true',
            'sim_mode': 'true',
            'sensors': 'false',
            'config_file': 'reseq_mk1_vcan.yaml',
        }.items(),
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name=package_name), 'launch', 'gazebo_launch.py')]
        )
    )

    return LaunchDescription(
        [
            rsp,
            gazebo_launch,
        ]
    )
