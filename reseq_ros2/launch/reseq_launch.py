from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory
import os
import sys
from ament_index_python.packages import get_package_share_directory

#Default config file path
share_folder = get_package_share_directory("reseq_ros2")
sys.path.append(share_folder+"/launch")
from common_functions_launch import *

#launch_setup is used through an OpaqueFunction because it is the only way to manipulate a command line argument directly in the launch file
def launch_setup(context, *args, **kwargs):
    #Get config path from command line, otherwise use the default path
    config_filename = LaunchConfiguration('config_file').perform(context)
    #Parse the config file
    # config = parse_config(f'{config_path}/{config_filename}')
    # addresses = get_addresses(config)
    # joints = get_joints(config)
    # endEffector = get_end_effector(config)
    launch_config = []

    # add optional nodes for sensors
    sensors_enabled = LaunchConfiguration('sensors').perform(context)
    digital_twin_enabled = LaunchConfiguration('d_twin').perform(context)
    is_can = LaunchConfiguration('is_can').perform(context) == 'true'

    # Core launch file
    core_launch_file = os.path.join(get_package_share_directory('reseq_ros2'), 'launch', 'reseq_core_launch.py')
    launch_config.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(core_launch_file),
        launch_arguments={
            'config_file': config_filename,
            'is_can': str(is_can).lower()
        }.items()
    ))

    # Sensor launch file
    if sensors_enabled == 'true':
        sensors_launch_file = os.path.join(get_package_share_directory('reseq_ros2'), 'launch', 'sensors_launch.py')
        launch_config.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch_file),
            launch_arguments={
                'config_file': config_filename,
            }.items()
        ))

    if digital_twin_enabled == 'true':
        # Digital twin launch file
        digital_twin_launch_file = os.path.join(get_package_share_directory('reseq_ros2'), 'launch', 'digital_twin_launch.py')
        launch_config.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(digital_twin_launch_file),
            launch_arguments={
                'config_file': config_filename
            }.items()
        ))
    
    return launch_config
    
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value = default_filename),
        DeclareLaunchArgument('is_can', default_value = 'true', description="is CAN or VCAN"),
        DeclareLaunchArgument('sensors', default_value = 'true', description="Enable sensors"),
        DeclareLaunchArgument('d_twin', default_value = 'true', description="Enable digital twin"),
        
        OpaqueFunction(function = launch_setup)
                             ])

