import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from reseq_ros2.utils.launch_utils import default_filename

share_folder = get_package_share_directory('reseq_description')


# launch_setup is used through an OpaqueFunction because it is the only way to manipulate a
# command line argument directly in the launch file
def launch_setup(context, *args, **kwargs):
    version = LaunchConfiguration('version').perform(context)
    # Get the configuration file path from command line, otherwise use the default path
    config_filename = LaunchConfiguration('config_file').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    external_log_level = LaunchConfiguration('external_log_level').perform(context)
    launch_config = []

    # add optional nodes for sensors
    sensors_enabled = LaunchConfiguration('sensors').perform(context)
    digital_twin_enabled = LaunchConfiguration('d_twin').perform(context)

    # use simulation time: should only be used with gazebo that's why default value is 'false'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)

    # Core launch file
    core_launch_file = os.path.join(
        get_package_share_directory('reseq_ros2'), 'launch', 'reseq_core_launch.py'
    )
    launch_config.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch_file),
            launch_arguments={
                'version': version,
                'config_file': config_filename,
                'log_level': log_level,
                'use_sim_time': use_sim_time,
            }.items(),
        )
    )

    # Sensor launch file
    if sensors_enabled == 'true':
        sensors_launch_file = os.path.join(
            get_package_share_directory('reseq_ros2'), 'launch', 'sensors_launch.py'
        )
        launch_config.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensors_launch_file),
                launch_arguments={
                    'version': version,
                    'config_file': config_filename,
                    'external_log_level': external_log_level,
                    'use_sim_time': use_sim_time,
                }.items(),
            )
        )

    if digital_twin_enabled == 'true':
        # Digital twin launch file
        digital_twin_launch_file = os.path.join(
            get_package_share_directory('reseq_ros2'), 'launch', 'digital_twin_launch.py'
        )
        launch_config.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(digital_twin_launch_file),
                launch_arguments={
                    'version': version,
                    'config_file': config_filename,
                    'log_level': log_level,
                    'external_log_level': external_log_level,
                    'use_sim_time': use_sim_time,
                    'sim_mode': sim_mode,
                }.items(),
            )
        )

    return launch_config


def generate_config_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    version = LaunchConfiguration('version').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    no_body_controllers = LaunchConfiguration('no_body_controllers').perform(context)
    no_arm_controllers = LaunchConfiguration('no_arm_controllers').perform(context)

    cmd = [
        'python3',
        os.path.join(share_folder, 'scripts/generate_configs.py'),
        config_file,
        '--version',
        version,
    ]
    if use_sim_time == 'true':
        cmd.append('--use_sim_time')
    if no_body_controllers == 'true':
        cmd.append('--no_body_controllers')
    if no_arm_controllers == 'true':
        cmd.append('--no_arm_controllers')

    generate_configs = ExecuteProcess(
        cmd=cmd,
        name='generate_configs',
        output='screen',
    )

    def on_process_exit(event, context):
        if event.returncode == 0:
            return [
                LogInfo(msg='Configuration files generated.'),
                OpaqueFunction(function=launch_setup),
            ]
        else:
            return [EmitEvent(event=Shutdown(reason='Configuration generation failed'))]

    return [
        generate_configs,
        # Wait for the config generation process to complete before proceeding
        RegisterEventHandler(
            OnProcessExit(target_action=generate_configs, on_exit=on_process_exit)
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('version', default_value='mk2', choices=['mk1', 'mk2']),
            DeclareLaunchArgument('config_file', default_value=default_filename),
            DeclareLaunchArgument('sensors', default_value='true', description='Enable sensors'),
            DeclareLaunchArgument(
                'd_twin', default_value='true', description='Enable digital twin'
            ),
            DeclareLaunchArgument(
                'log_level', default_value='info', description='Set log level for reseq nodes'
            ),
            DeclareLaunchArgument(
                'external_log_level',
                default_value='warn',
                description='Set log level for external nodes',
            ),
            # this argument is passed as 'true' by sim_launch.py file
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('sim_mode', default_value='false'),
            DeclareLaunchArgument('no_body_controllers', default_value='false'),
            DeclareLaunchArgument('no_arm_controllers', default_value='false'),
            OpaqueFunction(function=generate_config_setup),
        ]
    )
