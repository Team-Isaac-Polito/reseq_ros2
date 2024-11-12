import os
import sys
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    OpaqueFunction,
    ExecuteProcess,
    LogInfo,
    EmitEvent,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

# Default config file path
share_folder = get_package_share_directory("reseq_ros2")
sys.path.append(share_folder+"/launch")
from common_functions_launch import *

# launch_setup is used through an OpaqueFunction because it is the only way to manipulate a command line argument directly in the launch file
def launch_setup(context, *args, **kwargs):
    # Get the configuration file path from command line, otherwise use the default path
    config_filename = LaunchConfiguration('config_file').perform(context)
    launch_config = []

    # add optional nodes for sensors
    sensors_enabled = LaunchConfiguration('sensors').perform(context)
    digital_twin_enabled = LaunchConfiguration('d_twin').perform(context)

    # Core launch file
    core_launch_file = os.path.join(get_package_share_directory('reseq_ros2'), 'launch', 'reseq_core_launch.py')
    launch_config.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(core_launch_file),
        launch_arguments={
            'config_file': config_filename,
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
    config_file = LaunchConfiguration('config_file')
    
    generate_configs = ExecuteProcess(
        cmd=['python3', os.path.join(share_folder, 'scripts/generate_configs.py'), config_file],
        name='generate_configs',
        output='screen'
    )

    def on_process_exit(event, context):
        if event.returncode == 0:
            return [
                LogInfo(msg="Configuration files generated."),
                OpaqueFunction(function=launch_setup)
            ]
        else:
            return [
                EmitEvent(event=Shutdown(reason='Configuration generation failed'))
            ]

    return LaunchDescription([
        DeclareLaunchArgument('config_file',default_value=default_filename),
        DeclareLaunchArgument('sensors', default_value = 'true', description="Enable sensors"),
        DeclareLaunchArgument('d_twin', default_value = 'true', description="Enable digital twin"),
        generate_configs,
        # Wait for the config generation process to complete before proceeding
        RegisterEventHandler(
            OnProcessExit(
                target_action=generate_configs,
                on_exit=on_process_exit
            )
        )
    ])
