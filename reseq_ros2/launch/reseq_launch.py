from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch_ros.parameter_descriptions import ParameterFile
import yaml
from yaml import SafeLoader
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


# Default config file path
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

# launch_setup is used through an OpaqueFunction because it is the only way to manipulate a command line argument directly in the launch file
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
    launch_config.append(Node(
        package='reseq_ros2',
        executable='communication',
        name='communication',
        parameters=[{
            'can_channel': config['canbus']['channel'],
            'modules': addresses,
            'joints': joints,
            'end_effector': endEffector
        }],
        arguments=['--ros-args', '--log-level', log_level]
    ))
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
        }],
        arguments=['--ros-args', '--log-level', log_level]
    ))
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
        }],
        arguments=['--ros-args', '--log-level', log_level]
    ))
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
        }],
        arguments=['--ros-args', '--log-level', log_level]
    ))
    launch_config.append(Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_node',
        namespace="realsense",
        parameters=[ParameterFile(f"{config_path}/{config['realsense_config']}")],
        arguments=['--ros-args', '--log-level', 'warn']
    ))
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
            }],
            arguments=['--ros-args', '--log-level', log_level]
        ))
    robot_controllers = f"{config_path}/reseq_controllers.yaml"
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        arguments=['--ros-args', '--log-level', 'warn'],
    )
    launch_config.append(control_node)

    xacro_file = share_folder + "/description/robot.urdf.xacro"
    robot_description = xacro.process_file(xacro_file, mappings={'config_path': f'{config_path}/{config_filename}'}).toxml()
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}], # add other parameters here if required
        arguments=['--ros-args', '--log-level', 'warn']
    )
    launch_config.append(robot_state_publisher_node)

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", '--ros-args', '--log-level', 'warn'],
    )
    launch_config.append(joint_state_broadcaster_spawner)

    diff_controller_spawner1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller1", "--controller-manager", "/controller_manager", '--ros-args', '--log-level', 'warn'],
    )
    launch_config.append(diff_controller_spawner1)

    diff_controller_spawner2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller2", "--controller-manager", "/controller_manager", '--ros-args', '--log-level', 'warn'],
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

    launch_config.append(IncludeLaunchDescription(
        f"{get_package_share_directory('rplidar_ros')}/launch/rplidar_a2m8_launch.py",
        launch_arguments={'log_level': log_level}.items()
    ))
    return launch_config

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_filename),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function = launch_setup)
    ])
