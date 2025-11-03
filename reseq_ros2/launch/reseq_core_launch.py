from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from reseq_ros2.utils.launch_utils import (
    config_path,
    default_filename,
    get_addresses,
    parse_config,
)


# launch_setup is used through an OpaqueFunction because it is the only way to manipulate a
# command line argument directly in the launch file
def launch_setup(context, *args, **kwargs):
    version = LaunchConfiguration('version').perform(context)
    # Get config path from command line, otherwise use the default path
    config_filename = LaunchConfiguration('config_file').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(
        context
    )  # it's a string either 'true' or 'false'
    use_sim_time = True if use_sim_time == 'true' else False

    # Parse the config file
    config = parse_config(f'{config_path}/{version}/{config_filename}')
    addresses = get_addresses(config)
    launch_config = []
    launch_config.append(
        Node(
            package='reseq_ros2',
            executable='agevar',
            name='agevar',
            parameters=[
                {
                    'a': config['agevar_consts']['a'],
                    'b': config['agevar_consts']['b'],
                    'modules': addresses,
                    'use_sim_time': use_sim_time,
                }
            ],
            arguments=['--ros-args', '--log-level', log_level],
            on_exit=Shutdown(),
        )
    )
    launch_config.append(
        Node(
            package='reseq_ros2',
            executable='pivot_controller',
            name='pivot_controller',
            parameters=[
                {
                    'modules': addresses,
                    'use_sim_time': use_sim_time,
                }
            ],
            arguments=['--ros-args', '--log-level', log_level],
            on_exit=Shutdown(),
        )
    )

    launch_config.append(
        Node(
            package='reseq_ros2',
            executable='scaler',
            name='scaler',
            parameters=[
                {
                    'r_linear_vel': config['scaler_consts']['r_linear_vel'],
                    'r_inverse_radius': config['scaler_consts']['r_inverse_radius'],
                    'r_angular_vel': config['scaler_consts']['r_angular_vel'],
                    'version': config['version'],
                    'use_sim_time': use_sim_time,
                }
            ],
            arguments=['--ros-args', '--log-level', log_level],
            on_exit=Shutdown(),
        )
    )

    return launch_config


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('version', default_value='mk1', choices=['mk1', 'mk2']),
            DeclareLaunchArgument('config_file', default_value=default_filename),
            DeclareLaunchArgument('log_level', default_value='info'),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description="set use_sim_time to 'true' if you are using gazebo.\
                                    In general this parameter is not set from this launch\
                                    but instead is passed by other launch files that use this launch file.\
                                    Setting this arg to 'true', it will set the use_sim_time parameter of all nodes launched in this file \
                                    to True.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
