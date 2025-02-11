from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

from reseq_ros2.utils.launch_utils import config_path, default_filename, parse_config


# launch_setup is used through an OpaqueFunction because it is the only way to manipulate a command
# line argument directly in the launch file
def launch_setup(context, *args, **kwargs):
    # Get config path from command line, otherwise use the default path
    config_filename = LaunchConfiguration('config_file').perform(context)
    # Parse the config file
    config = parse_config(f'{config_path}/{config_filename}')

    launch_config = []

    # for each sensor in the config file
    for sensor in config['sensors']:
        # name of the sensor
        name = list(sensor.keys())[0]
        # if sensor == True, it means that in the config file the sensor has to activated
        if sensor[name]:
            # now a series of if
            if name == 'lidar':
                launch_config.append(
                    IncludeLaunchDescription(
                        f'{get_package_share_directory("rplidar_ros")}'
                        '/launch/rplidar_a2m8_launch.py'
                    )
                )
            if name == 'realsense':
                launch_config.append(
                    Node(
                        package='realsense2_camera',
                        executable='realsense2_camera_node',
                        name='realsense2_camera_node',
                        namespace='realsense',
                        parameters=[ParameterFile(f"{config_path}/{config['realsense_config']}")],
                        arguments=['--ros-args', '--log-level', 'warn'],
                    )
                )
            # Launch a usb_cam node for each usb_camera present
            if name == 'usb_cameras':
                num_usb_cam = sensor[name] 
                for i in range(0, num_usb_cam):
                    usb_cam_config=f"usb_camera_config_{i}"
                    launch_config.append(
                        Node(
                            package='usb_cam',
                            executable='usb_cam_node_exe',
                            name=f"usb_cam_{i}",
                            namespace=f"usb_cam_{i}",
                            parameters=[ParameterFile(f"{config_path}/{usb_cam_config}")],
                            arguments=['--ros-args'],
                        )
                    )

    return launch_config


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('config_file', default_value=default_filename),
            OpaqueFunction(function=launch_setup),
        ]
    )
