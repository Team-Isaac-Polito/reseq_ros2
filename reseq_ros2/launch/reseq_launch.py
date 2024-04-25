from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
import yaml
from yaml import SafeLoader
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

share_folder = get_package_share_directory("reseq_ros2")
default_path = "/config/reseq_mk1_can.yaml"

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

def get_end_effectors(config):
    l = []
    for mod in config['modules']:
        if mod['hasEndEffector']:
            l.append(mod['address'])
    return l
    
def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_path').perform(context)
    config = parse_config(share_folder + config_path)
    addresses = get_addresses(config)
    joints = get_joints(config)
    endEffectors = get_end_effectors(config)
    nodes = []
    nodes.append(Node(
            package='reseq_ros2',
            executable='communication',
            name='communication',
            parameters=[{
                'can_channel': config['canbus']['channel'],
                'modules': addresses,
                'joints': joints,
                'end_effectors': endEffectors
            }]))
    nodes.append(Node(
            package='reseq_ros2',
            executable='agevar',
            name='agevar',
            parameters=[{
                'a': config['dimensions']['a'],
                'b': config['dimensions']['b'],
                'd': config['dimensions']['d'],
                'r_eq': config['dimensions']['r_eq'],
                'modules': addresses,
                'joints': joints,
                'end_effectors': endEffectors
            }]))
    nodes.append(Node(
            package='reseq_ros2',
            executable='scaler',
            name='scaler',
            parameters=[{
                'r_linear_vel': config['dimensions']['r_linear_vel'],
                'r_inverse_radius': config['dimensions']['r_inverse_radius'],
                'r_pitch_vel': config['dimensions']['r_pitch_vel'],
                'r_head_pitch_vel': config['dimensions']['r_head_pitch_vel'],
                'r_head_yaw_vel': config['dimensions']['r_head_yaw_vel'],
            }]))
    nodes.append(Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            parameters=[{
                'config_file': share_folder + "/config/realsense_rgb_motion.yaml"
            }]))
    if config['version'] == 'mk1':
        nodes.append(Node(
                package='reseq_ros2',
                executable='enea',
                name='enea',
                parameters=[{
                    'pitch': config['enea_consts']['i_pitch'],
                    'head_pitch': config['enea_consts']['i_head_pitch'],
                    'head_yaw': config['enea_consts']['i_head_yaw'],
                    'servo_speed': config['enea_consts']['servo_speed'],
                    'r_pitch': config['enea_consts']['r_pitch'],
                    'r_head_pitch': config['enea_consts']['r_head_pitch'],
                    'r_head_yaw': config['enea_consts']['r_head_yaw'],
                    'pitch_conv': config['enea_consts']['pitch_conv'],
                    'end_effectors': endEffectors
                }]))
    return nodes
    
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config_path', default_value=default_path), 
                             OpaqueFunction(function = launch_setup)
                             ])

 #Node(
        #    package='reseq_ros2',
        #    executable='remote_test',
        #    name='remote_test',
        #),