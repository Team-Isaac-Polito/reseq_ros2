from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from reseq_ros2.utils.launch_utils import (
    config_path,
    default_filename,
    get_addresses,
    get_end_effector,
    get_joints,
    parse_config,
)


# launch_setup is used through an OpaqueFunction because it is the only way to manipulate a
# command line argument directly in the launch file
def launch_setup(context, *args, **kwargs):
    # Get config path from command line, otherwise use the default path
    config_filename = LaunchConfiguration('config_file').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    # Parse the config file
    config = parse_config(f'{config_path}/{config_filename}')
    addresses = get_addresses(config)
    joints = get_joints(config)
    endEffector = get_end_effector(config)
    launch_config = []

    # check if it is can
    if config['canbus']['channel'].startswith('can'):
        launch_config.append(
            Node(
                package='reseq_ros2',
                executable='communication',
                name='communication',
                parameters=[
                    {
                        'can_channel': config['canbus']['channel'],
                        'modules': addresses,
                        'joints': joints,
                        'end_effector': endEffector,
                    }
                ],
                arguments=['--ros-args', '--log-level', log_level],
                on_exit=Shutdown(),
            )
        )
    else:
        launch_config.append(
            Node(
                package='reseq_ros2',
                executable='communication',
                name='communication',
                parameters=[
                    {
                        'can_channel': config['canbus']['channel'],
                        'modules': addresses,
                        'joints': joints,
                        'end_effector': endEffector,
                    }
                ],
                arguments=['--ros-args', '--log-level', log_level],
            )
        )
    launch_config.append(
        Node(
            package='reseq_ros2',
            executable='agevar',
            name='agevar',
            parameters=[
                {
                    'a': config['agevar_consts']['a'],
                    'b': config['agevar_consts']['b'],
                    'd': config['agevar_consts']['d'],
                    'r_eq': config['agevar_consts']['r_eq'],
                    'modules': addresses,
                    'joints': joints,
                    'end_effector': endEffector,
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
                    'd': config['agevar_consts']['d'],
                    'r_eq': config['agevar_consts']['r_eq'],
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
                    **{
                        'r_linear_vel': config['scaler_consts']['r_linear_vel'],
                        'r_inverse_radius': config['scaler_consts']['r_inverse_radius'],
                        'r_angular_vel': config['scaler_consts']['r_angular_vel'],
                        'version': config['version'],
                    },
                    **(
                        {
                            'r_pitch_vel': config['scaler_consts']['r_pitch_vel'],
                            'r_head_pitch_vel': config['scaler_consts']['r_head_pitch_vel'],
                            'r_head_roll_vel': config['scaler_consts']['r_head_roll_vel'],
                        }
                        if config['version'] == 'mk1'
                        else {}
                    ),
                }
            ],
            arguments=['--ros-args', '--log-level', log_level],
            on_exit=Shutdown(),
        )
    )

    if config['version'] == 'mk1':
        launch_config.append(
            Node(
                package='reseq_ros2',
                executable='enea',
                name='enea',
                parameters=[
                    {
                        'pitch': config['enea_consts']['i_pitch'],
                        'head_pitch': config['enea_consts']['i_head_pitch'],
                        'head_roll': config['enea_consts']['i_head_roll'],
                        'servo_speed': config['enea_consts']['servo_speed'],
                        'r_pitch': config['enea_consts']['r_pitch'],
                        'r_head_pitch': config['enea_consts']['r_head_pitch'],
                        'r_head_roll': config['enea_consts']['r_head_roll'],
                        'pitch_conv': config['enea_consts']['pitch_conv'],
                        'end_effector': endEffector,
                    }
                ],
                arguments=['--ros-args', '--log-level', log_level],
                on_exit=Shutdown(),
            )
        )

    return launch_config
    # return launch_config


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('config_file', default_value=default_filename),
            DeclareLaunchArgument('log_level', default_value='info'),
            OpaqueFunction(function=launch_setup),
        ]
    )