from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import sys
import yaml
from yaml import SafeLoader
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

share_folder = get_package_share_directory("reseq_ros2")

def get_config():
    args = sys.argv
    if len(args)>1:
        filename = share_folder + args[1]
    else:
        filename = share_folder + "/config/reseq_mk1_can.yaml"
    return filename

def parse_config(filename):
    with open(filename) as f:
        return yaml.load(f, Loader=SafeLoader)
    
def declare_parameters(config):
    #Declare each parameter present in the yaml fle as a Launch Argument
    args = []
    for k,v in config.items():
        if k=='geometric_consts':
            for name, val in v.items():
                args.append(DeclareLaunchArgument(name, default_value = str(val)))
        elif k=='enea_consts':
            for name, val in v.items():
                args.append(DeclareLaunchArgument(name, default_value = str(val)))
        elif k=='modules':
            addresses = []
            joints = []
            endEffectors = []
            for mod in v:
                addresses.append(mod['address'])
                if mod['hasJoint']:
                    joints.append(mod['address'])
                if mod['hasEndEffector']:
                    endEffectors.append(mod['address'])
            args.append(DeclareLaunchArgument('modules', default_value = str(addresses)))
            args.append(DeclareLaunchArgument('joints', default_value = str(joints)))
            args.append(DeclareLaunchArgument('end_effectors', default_value = str(endEffectors)))
        elif k=='canbus':
            args.append(DeclareLaunchArgument('can_channel', default_value = v['channel']))
    return args

def generate_launch_description():
    config = parse_config(get_config())
    ld =  LaunchDescription(declare_parameters(config) + [
        Node(
            package='reseq_ros2',
            executable='communication',
            name='communication',
            parameters=[{
                'can_channel': LaunchConfiguration('can_channel'),
                'modules': LaunchConfiguration('modules'),
                'joints': LaunchConfiguration('joints'),
                'end_effectors': LaunchConfiguration('end_effectors')
            }]
        ),
        Node(
            package='reseq_ros2',
            executable='agevar',
            name='agevar',
            parameters=[{
                'a': LaunchConfiguration('a'),
                'b': LaunchConfiguration('b'),
                'd': LaunchConfiguration('d'),
                'r_eq': LaunchConfiguration('r_eq'),
                'modules': LaunchConfiguration('modules'),
                'joints': LaunchConfiguration('joints'),
                'end_effectors': LaunchConfiguration('end_effectors')
            }]
        ),
        Node(
            package='reseq_ros2',
            executable='scaler',
            name='scaler',
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            parameters=[{
                'config_file': share_folder + "/config/realsense_rgb_motion.yaml"
            }]
        )
        #Node(
        #    package='reseq_ros2',
        #    executable='remote_test',
        #    name='remote_test',
        #),
    ])
    #Launch EnEA conditionally to the version
    if config['version'] == 'mk1':
        ld.add_action(
            Node(
                package='reseq_ros2',
                executable='enea',
                name='enea',
                parameters=[{
                    'pitch': LaunchConfiguration('i_pitch'),
                    'head_pitch': LaunchConfiguration('i_head_pitch'),
                    'head_yaw': LaunchConfiguration('i_head_yaw'),
                    'servo_speed': LaunchConfiguration('servo_speed'),
                    'r_pitch': LaunchConfiguration('r_pitch'),
                    'r_head_pitch': LaunchConfiguration('r_head_pitch'),
                    'r_head_yaw': LaunchConfiguration('r_head_yaw'),
                    'pitch_conv': LaunchConfiguration('pitch_conv'),
                    'end_effectors': LaunchConfiguration('end_effectors')
                }]
        ))
    return ld
