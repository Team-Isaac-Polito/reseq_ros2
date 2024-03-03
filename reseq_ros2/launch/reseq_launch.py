from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='reseq_ros2',
            executable='communication',
            name='communication',
        ),
        Node(
            package='reseq_ros2',
            executable='agevar',
            name='agevar',
        ),
        Node(
            package='reseq_ros2',
            executable='enea',
            name='enea',
        ),
        Node(
            package='reseq_ros2',
            executable='scaler',
            name='scaler',
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
        )
        #Node(
        #    package='reseq_ros2',
        #    executable='remote_test',
        #    name='remote_test',
        #),
    ])

