import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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

share_folder = get_package_share_directory('reseq_ros2')


# launch_setup is used through an OpaqueFunction because it is the only way to manipulate a
# command line argument directly in the launch file
def launch_setup(context, *args, **kwargs):
    # Get config path from command line, otherwise use the default path
    config_filename = LaunchConfiguration('config_file').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    external_log_level = LaunchConfiguration('external_log_level').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) # it's a string either 'true' or 'false'
    
    # Parse the config file
    config = parse_config(f'{config_path}/{config_filename}')
    addresses = get_addresses(config)
    joints = get_joints(config)
    endEffector = get_end_effector(config)
    launch_config = []

    robot_controllers = f'{config_path}/reseq_controllers.yaml'
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='both',
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
        arguments=['--ros-args', '--log-level', external_log_level],
    )
    launch_config.append(control_node)

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--ros-args',
            '--log-level',
            external_log_level,
        ],
    )
    launch_config.append(joint_state_broadcaster_spawner)

    num_modules = config.get('num_modules', 0)
    for i in range(num_modules):
        module_controller = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                f'diff_controller{i + 1}',
                '--controller-manager',
                '/controller_manager',
                '--ros-args',
                '--log-level',
                external_log_level,
            ],
        )
        launch_config.append(module_controller)

    xacro_file = share_folder + '/description/robot.urdf.xacro'
    robot_description = xacro.process_file(
        xacro_file, mappings={'config_path': f'{config_path}/{config_filename}'}
    ).toxml()
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description}
        ],  # add other parameters here if required
        arguments=['--ros-args', '--log-level', external_log_level],
    )
    launch_config.append(robot_state_publisher_node)

    if config['canbus']['channel'].startswith('vcan'):
        feedback_replicator = Node(
            package='reseq_ros2',
            executable='feedback_replicator',
            name='feedback_replicator',
            parameters=[
                {
                    'modules': addresses,
                    'joints': joints,
                    'end_effector': endEffector,
                    'use_sim_time': use_sim_time=='true',
                }
            ],
            arguments=['--ros-args', '--log-level', log_level],
        )
        launch_config.append(feedback_replicator)

    return launch_config


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('config_file', default_value=default_filename),
            DeclareLaunchArgument('log_level', default_value='info'),
            DeclareLaunchArgument('external_log_level', default_value='warn'),
            DeclareLaunchArgument('use_sim_time', 
                                  default_value='false',
                                  description="set use_sim_time to 'true' if you are using gazebo.\
                                    In general this parameter is not set from this launch\
                                    but instead is passed by other launch files that use this launch file.\
                                    Setting this arg to 'true', it will set the use_sim_time parameter of all nodes launched in this file \
                                    to True."),
            OpaqueFunction(function=launch_setup),
        ]
    )
