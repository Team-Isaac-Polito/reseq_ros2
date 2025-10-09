from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='image_transport',
                executable='republish',
                name='jpeg_republisher',
                arguments=[
                    'raw',
                    'compressed',
                ],
                remappings=[
                    ('in', '/detector/model_output'),
                    ('out/compressed', '/detector/model_output/compressed'),
                ],
                output='screen',
            ),
            Node(
                package='reseq_ros2',
                executable='detector',
                name='detector',
                output='screen',
            ),
        ]
    )
