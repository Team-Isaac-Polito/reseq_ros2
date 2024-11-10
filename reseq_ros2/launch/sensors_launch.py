from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import sys

#Default config file path
share_folder = get_package_share_directory("reseq_ros2")
sys.path.append(share_folder+"/launch")
from common_functions_launch import *

#launch_setup is used through an OpaqueFunction because it is the only way to manipulate a command line argument directly in the launch file
def launch_setup(context, *args, **kwargs):
    #Get config path from command line, otherwise use the default path
    config_filename = LaunchConfiguration('config_file').perform(context)
    #Parse the config file
    config = parse_config(f'{config_path}/{config_filename}')
    sensors_config = parse_config(f'{config_path}/sensors_config.yaml')

    launch_config = []

    for sensor in sensors_config['sensors']:
        # if sensor doens't have a launch file, create the Node, otherwise use it
        # YAML value true are read as boolean, so don't use equality check "true"
        if sensor['enabled']:
            if sensor['has_launch_file']:
                launch_config.append(IncludeLaunchDescription(
                    f"{get_package_share_directory(sensor['package'])}/launch/{sensor['launch_file']}"))
            else:
                # if sensor has parameters, it is contained in the config
                if sensor['has_parameter']:
                    launch_config.append(Node(
                    package=sensor['package'],
                    executable=sensor['executable'],
                    name=sensor['name'],
                    namespace=sensor['namespace'],
                    parameters=[ParameterFile(f"{config_path}/{config['realsense_config']}")]))
                else:
                    launch_config.append(Node(
                    package=sensor['package'],
                    executable=sensor['executable'],
                    name=sensor['name'],
                    namespace=sensor['namespace']))
    return launch_config
    
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config_file', default_value = default_filename),
                             OpaqueFunction(function = launch_setup)
                             ])
