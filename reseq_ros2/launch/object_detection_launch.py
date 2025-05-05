from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='reseq_ros2',
                executable='detection_data_handler',
                name='detection_data_handler',
                output='screen',
                parameters=[{'output_path': 'object_detections.csv'}],
            )
        ]
    )
