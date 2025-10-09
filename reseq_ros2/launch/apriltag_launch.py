import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def launch_setup(context, *args, **kwargs):
    use_webcam = LaunchConfiguration('use_webcam').perform(context)
    config_path = os.path.join(get_package_share_directory('reseq_ros2'), 'config')

    if use_webcam == 'true':
        remappings = [
            ('image_rect', '/webcam/image_raw'),
            ('camera_info', '/webcam/camera_info'),
            ('detections', '/tag_detections'),
        ]
    else:
        remappings = [
            ('image_rect', '/realsense/realsense2_camera_node/color/image_raw'),
            ('camera_info', '/realsense/realsense2_camera_node/color/camera_info'),
            ('detections', '/tag_detections'),
        ]

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=remappings,
        parameters=[ParameterFile(f'{config_path}/apriltag.yaml')],
    )

    return [apriltag_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_webcam',
                default_value='false',
                description='Use webcam instead of robot sensors',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
