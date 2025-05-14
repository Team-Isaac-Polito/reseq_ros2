from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='reseq_ros2',
                executable='batch_data_handler',
                name='batch_data_handler',
                output='screen',
                parameters=[{'team_name': 'ISAAC'}, {'country': 'Italy'}, {'mission': 1}],
            )
        ]
    )
