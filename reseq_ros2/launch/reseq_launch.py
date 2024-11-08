import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction, ExecuteProcess, LogInfo
from launch_ros.parameter_descriptions import ParameterFile
from launch.event_handlers import OnProcessExit
import yaml
from yaml import SafeLoader
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

# Default config file path
share_folder = get_package_share_directory("reseq_ros2")
config_path = f'{share_folder}/config'
temp_config_path = os.path.join(config_path, 'temp')
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
    # Get the configuration file path from command line, otherwise use the default path
    config_filename = LaunchConfiguration('config_file').perform(context)

    # Parse the main configuration file
    config = parse_config(f'{temp_config_path}/{config_filename}')

    # Extract relevant configurations
    addresses = get_addresses(config)
    joints = get_joints(config)
    endEffector = get_end_effector(config)

    # List of nodes to launch with their respective parameters
    launch_config = []
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
    launch_config.append(Node(
            package='reseq_ros2',
            executable='joint_publisher',
            name='joint_publisher',
            parameters=[{
                'modules': addresses,
                'joints': joints,
                'end_effector': endEffector,
                'arm_pitch_origin': config['joint_pub_consts']['arm_pitch_origin'],
                'head_pitch_origin': config['joint_pub_consts']['head_pitch_origin'],
                'head_roll_origin': config['joint_pub_consts']['head_roll_origin'],
                'vel_gain': config['joint_pub_consts']['vel_gain'],
                'arm_pitch_gain': config['joint_pub_consts']['arm_pitch_gain'],
                'b': config['agevar_consts']['b'],
            }]))
    launch_config.append(Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            namespace="realsense",
            parameters=[ParameterFile(f"{temp_config_path}/{config['realsense_config']}")]))
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
    robot_controllers = f"{temp_config_path}/reseq_controllers.yaml"
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
    
    xacro_file = share_folder + "/description/robot.urdf.xacro"
    robot_description = xacro.process_file(xacro_file, mappings={'config_path': f'{temp_config_path}/{config_filename}'}).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}] # add other parameters here if required
    )
    launch_config.append(robot_state_publisher_node)

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    launch_config.append(joint_state_broadcaster_spawner)

    num_modules = config.get('num_modules', 0)
    for i in range(num_modules):
        launch_config.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=[f"diff_controller{i + 1}", "--controller-manager", "/controller_manager"],
            )
    )

    # frf = Node(
    #     package='reseq_ros2',
    #     executable='fake_robot_feedback',
    #     name='fake_robot_feedback',
    #     parameters=[{
    #         'modules': addresses,
    #     }]
    # )
    # launch_config.append(frf)

    launch_config.append(IncludeLaunchDescription(
        f"{get_package_share_directory('rplidar_ros')}/launch/rplidar_a2m8_launch.py"
    ))
    return launch_config
    
def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    
    generate_configs = ExecuteProcess(
        cmd=['python3', os.path.join(share_folder, 'scripts/generate_configs.py'), config_file],
        name='generate_configs',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_filename,
        ),
        generate_configs,
        # Wait for the config generation process to complete before proceeding
        RegisterEventHandler(
            OnProcessExit(
                target_action=generate_configs,
                on_exit=[LogInfo(msg="Configuration files generated."),
                         OpaqueFunction(function=launch_setup)
                ]
            )
        )
    ])

