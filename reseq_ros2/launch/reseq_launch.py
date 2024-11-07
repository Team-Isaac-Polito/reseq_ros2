from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction
import yaml
from yaml import SafeLoader
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

#Default config file path
share_folder = get_package_share_directory("reseq_ros2")
config_path = f'{share_folder}/config'
default_filename = "reseq_mk1_can.yaml"

def parse_config(filename):
    with open(filename) as f:
        return yaml.load(f, Loader=SafeLoader)
    
def get_addresses(config):
    l = []
    for mod in config['modules']:
        l.append(mod['address'])
    return l

def get_joints(config):
    l = []
    for mod in config['modules']:
        if mod['hasJoint']:
            l.append(mod['address'])
    return l

def get_end_effector(config):
    for mod in config['modules']:
        if mod['hasEndEffector']:
            return mod['address']

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
    lidar_enabled = LaunchConfiguration('lidar').perform(context) == 'true'
    camera_enabled = LaunchConfiguration('realsense').perform(context) == 'true'

    # Core launch file
    core_launch_file = os.path.join(get_package_share_directory('reseq_ros2'), 'launch', 'reseq_core_launch.py')
    launch_config.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(core_launch_file),
        launch_arguments={
            'config_file': config_filename
        }.items()
    ))

    # Sensor launch file
    sensors_launch_file = os.path.join(get_package_share_directory('reseq_ros2'), 'launch', 'sensors_launch.py')
    launch_config.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensors_launch_file),
        launch_arguments={
            'config_file': config_filename,
            'lidar': str(lidar_enabled).lower(),
            'realsense': str(camera_enabled).lower()
        }.items()
    ))

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
        DeclareLaunchArgument('lidar', default_value = 'true', description="Enable lidar sensor"),
        DeclareLaunchArgument('realsense', default_value = 'true', description="Enable realsense camera"),
        
        OpaqueFunction(function = launch_setup)
                             ])

