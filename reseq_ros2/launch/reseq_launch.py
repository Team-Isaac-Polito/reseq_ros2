from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_folder = get_package_share_directory("reseq_ros2")
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
            parameters=[{
                'config_file': share_folder + "/config/realsense_rgb_motion.yaml"
            }]
        )
        #Node(
        #    package='reseq_ros2',
        #    executable='remote_test',
        #    name='remote_test',
        #),
    ])

