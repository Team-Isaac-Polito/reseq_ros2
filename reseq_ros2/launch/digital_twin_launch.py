from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import xacro
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
    
    xacro_file = share_folder + "/description/robot.urdf.xacro"
    robot_description = xacro.process_file(xacro_file, mappings={'config_path': f'{config_path}/{config_filename}'}).toxml()
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}] # add other parameters here if required
    )
    launch_config.append(robot_state_publisher_node)


    # frf = Node(
    #     package='reseq_ros2',
    #     executable='fake_robot_feedback',
    #     name='fake_robot_feedback',
    #     parameters=[{
    #         'modules': addresses,
    #     }]
    # )
    # launch_config.append(frf)

    # launch_config.append(IncludeLaunchDescription(
    #     f"{get_package_share_directory('rplidar_ros')}/launch/rplidar_a2m8_launch.py"
    # ))

    return launch_config
    
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value = default_filename),
        
        OpaqueFunction(function = launch_setup)
                             ])

