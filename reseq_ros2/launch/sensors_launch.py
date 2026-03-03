import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

from reseq_ros2.utils.launch_utils import config_path, default_filename, parse_config


def _stop_rplidar_on_shutdown(event, context):
    """Stop the RPLIDAR motor when the launch system shuts down.

    Sends STOP scan + SET_MOTOR_PWM=0 to halt the motor, preventing it from
    spinning indefinitely after a container/launch shutdown.
    """
    import struct
    import time

    port = '/dev/ttyUSB0'
    try:
        import serial

        s = serial.Serial()
        s.port = port
        s.baudrate = 115200
        s.timeout = 1
        s.dsrdtr = False
        s.dtr = False
        s.open()
        # STOP scan command
        s.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        # SET_MOTOR_PWM = 0
        pwm_data = struct.pack('<H', 0)
        header = bytes([0xA5, 0xF0, len(pwm_data)])
        full_cmd = header + pwm_data
        checksum = 0
        for b in full_cmd:
            checksum ^= b
        full_cmd += bytes([checksum])
        s.write(full_cmd)
        time.sleep(0.1)
        s.dtr = False
        s.close()
        print('[sensors_launch] RPLIDAR motor stopped.')
    except Exception as e:
        print(f'[sensors_launch] Could not stop RPLIDAR motor: {e}')


# launch_setup is used through an OpaqueFunction because it is the only way to manipulate a command
# line argument directly in the launch file
def launch_setup(context, *args, **kwargs):
    version = LaunchConfiguration('version').perform(context)
    # Get config path from command line, otherwise use the default path
    config_filename = LaunchConfiguration('config_file').perform(context)
    # Parse the config file
    config = parse_config(f'{config_path}/{version}/{config_filename}')

    external_log_level = LaunchConfiguration('external_log_level').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

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
                        '/launch/rplidar_a2m8_launch.py',
                        launch_arguments={
                            'frame_id': 'laser_frame',
                            'use_sim_time': use_sim_time,
                        }.items(),
                    ),
                )
                # Register shutdown handler to stop RPLIDAR motor on exit
                launch_config.append(
                    RegisterEventHandler(
                        OnShutdown(on_shutdown=_stop_rplidar_on_shutdown),
                    )
                )
            if name == 'realsense':
                launch_config.append(
                    Node(
                        package='realsense2_camera',
                        executable='realsense2_camera_node',
                        name='realsense2_camera_node',
                        namespace='realsense',
                        parameters=[
                            ParameterFile(f'{config_path}/{config["realsense_config"]}'),
                            {
                                'use_sim_time': use_sim_time == 'true',
                            },
                        ],
                        arguments=['--ros-args', '--log-level', external_log_level],
                    )
                )
            # Launch a usb_cam node for each usb_camera present
            if name == 'usb_cameras':
                num_usb_cam = sensor[name]
                for i in range(0, num_usb_cam):
                    usb_cam_config = f'usb_camera_config_{i}'
                    if os.path.exists(f'{config_path}/{usb_cam_config}') is False:
                        break
                    launch_config.append(
                        Node(
                            package='usb_cam',
                            executable='usb_cam_node_exe',
                            name=f'usb_cam_{i}',
                            namespace=f'usb_cam_{i}',
                            parameters=[
                                ParameterFile(f'{config_path}/{usb_cam_config}'),
                                {
                                    '.image_raw.jpeg_quality': 30,
                                    '.image_raw.png_level': 3,
                                },
                            ],
                            arguments=['--ros-args', '--log-level', external_log_level],
                        )
                    )

    return launch_config


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('version', default_value='mk1', choices=['mk1', 'mk2']),
            DeclareLaunchArgument('config_file', default_value=default_filename),
            DeclareLaunchArgument('external_log_level', default_value='warn'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            OpaqueFunction(function=launch_setup),
        ]
    )
