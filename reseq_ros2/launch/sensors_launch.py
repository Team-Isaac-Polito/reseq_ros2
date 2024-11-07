from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch_ros.parameter_descriptions import ParameterFile
import yaml
from yaml import SafeLoader
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

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
    config = parse_config(f'{config_path}/{config_filename}')
    # Sensor arguments
    lidar_enabled = LaunchConfiguration('lidar').perform(context)
    realsense_enabled = LaunchConfiguration('realsense').perform(context)

    launch_config = []

    if realsense_enabled == 'true':
        launch_config.append(Node(
                package='realsense2_camera',
                executable='realsense2_camera_node',
                name='realsense2_camera_node',
                namespace="realsense",
                parameters=[ParameterFile(f"{config_path}/{config['realsense_config']}")]))
    if lidar_enabled == "true":
        launch_config.append(IncludeLaunchDescription(
            f"{get_package_share_directory('rplidar_ros')}/launch/rplidar_a2m8_launch.py"
        ))
    return launch_config
    
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config_file', default_value = default_filename),
                            DeclareLaunchArgument('lidar', default_value = 'true', description="Enable lidar sensor"),
                            DeclareLaunchArgument('realsense', default_value = "true", description="Enable realsense camera"),
                             OpaqueFunction(function = launch_setup)
                             ])
