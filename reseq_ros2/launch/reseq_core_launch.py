from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction, RegisterEventHandler, LogInfo, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
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
    config = parse_config(f'{config_path}/{config_filename}')
    addresses = get_addresses(config)
    joints = get_joints(config)
    endEffector = get_end_effector(config)
    launch_config = []
    core_nodes = [] # it contains indexes in launch_config

    # check if it is can
    if config['canbus']['channel'].startswith("can"):
        core_nodes.append(0)

    launch_config.append(Node(
                package='reseq_ros2',
                executable='communication',
                name='communication',
                parameters=[{
                    'can_channel': config['canbus']['channel'],
                    'modules': addresses,
                    'joints': joints,
                    'end_effector': endEffector
                }]))
    launch_config.append(Node(
            package='reseq_ros2',
            executable='agevar',
            name='agevar',
            parameters=[{
                'a': config['agevar_consts']['a'],
                'b': config['agevar_consts']['b'],
                'd': config['agevar_consts']['d'],
                'r_eq': config['agevar_consts']['r_eq'],
                'modules': addresses,
                'joints': joints,
                'end_effector': endEffector
            }]))
    launch_config.append(Node(
            package='reseq_ros2',
            executable='scaler',
            name='scaler',
            parameters=[{
                'r_linear_vel': config['scaler_consts']['r_linear_vel'],
                'r_inverse_radius': config['scaler_consts']['r_inverse_radius'],
                'r_pitch_vel': config['scaler_consts']['r_pitch_vel'],
                'r_head_pitch_vel': config['scaler_consts']['r_head_pitch_vel'],
                'r_head_roll_vel': config['scaler_consts']['r_head_roll_vel'],
            }]))

    if config['version'] == 'mk1':
        launch_config.append(Node(
                package='reseq_ros2',
                executable='enea',
                name='enea',
                parameters=[{
                    'pitch': config['enea_consts']['i_pitch'],
                    'head_pitch': config['enea_consts']['i_head_pitch'],
                    'head_roll': config['enea_consts']['i_head_roll'],
                    'servo_speed': config['enea_consts']['servo_speed'],
                    'r_pitch': config['enea_consts']['r_pitch'],
                    'r_head_pitch': config['enea_consts']['r_head_pitch'],
                    'r_head_roll': config['enea_consts']['r_head_roll'],
                    'pitch_conv': config['enea_consts']['pitch_conv'],
                    'end_effector': endEffector
                }]))
    robot_controllers = f"{config_path}/reseq_controllers.yaml"
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    launch_config.append(control_node)

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    launch_config.append(joint_state_broadcaster_spawner)

    diff_controller_spawner1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller1", "--controller-manager", "/controller_manager"],
    )
    launch_config.append(diff_controller_spawner1)

    diff_controller_spawner2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller2", "--controller-manager", "/controller_manager"],
    )
    launch_config.append(diff_controller_spawner2)

    # frf = Node(
    #     package='reseq_ros2',
    #     executable='fake_robot_feedback',
    #     name='fake_robot_feedback',
    #     parameters=[{
    #         'modules': addresses,
    #     }]
    # )
    # launch_config.append(frf)

    # all other reseq_core nodes are core nodes
    core_nodes += list(range(1, len(launch_config)))

    event_handlers = []
    # create list of event handlers for each core node
    for el in core_nodes:
        event_handlers.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action = launch_config[el],
                    on_exit=[
                        LogInfo(msg=f'{launch_config[el].name} exited, system ends'),
                        EmitEvent(event=Shutdown())
                    ]
                )
            )
        )
    
    return launch_config + event_handlers
    # return launch_config
    
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value = default_filename),
        
        OpaqueFunction(function = launch_setup)
                             ])