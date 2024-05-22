from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch_ros.parameter_descriptions import ParameterFile
import yaml
from yaml import SafeLoader
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

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
    addresses = get_addresses(config)
    joints = get_joints(config)
    endEffector = get_end_effector(config)
    nodes = []
    nodes.append(Node(
            package='reseq_ros2',
            executable='communication',
            name='communication',
            parameters=[{
                'can_channel': config['canbus']['channel'],
                'modules': addresses,
                'joints': joints,
                'end_effector': endEffector
            }]))
    nodes.append(Node(
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
    nodes.append(Node(
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
    nodes.append(Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            namespace="realsense",
            parameters=[ParameterFile(f"{config_path}/{config['realsense_config']}")]))
    if config['version'] == 'mk1':
        nodes.append(Node(
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
    
    xacro_file = share_folder + "/description/robot.urdf.xacro"
    robot_description = xacro.process_file(xacro_file, mappings={'config_path': f'{config_path}/{config_filename}'}).toxml()
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}] # add other parameters here if required
    )
    nodes.append(robot_state_publisher_node)
    return nodes
    
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config_file', default_value = default_filename), 
                             OpaqueFunction(function = launch_setup)
                             ])

 #Node(
        #    package='reseq_ros2',
        #    executable='remote_test',
        #    name='remote_test',
        #),