import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):

    launch_config = []

    # TAKE ARGUMENTS FROM CONTEXT AND APPLY TRANSFORMATIONS IF NEEDED
    use_sim_time_arg = LaunchConfiguration('use_sim_time').perform(
        context=context
    )  # 'true' or 'false'
    use_sim_time = (
        True if use_sim_time_arg == 'true' else False
    )  # boolean version of use_sim_time_arg

    controllers_config_file = os.path.join(
        get_package_share_directory('reseq_description'),
        'config',
        'temp',
        'reseq_controllers.yaml',
    )

    # Load all MoveIt configuration files
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='simplified_arm_assembly',  # This name comes from your SRDF
            package_name='reseq_arm_mk2',
        )
        .robot_description(
            file_path='urdf/reseq_arm_mk2.xacro',
            # We must pass the same arguments as the robot_state_publisher
            mappings={
                'sim_mode': use_sim_time_arg,
                'controllers_config_file': controllers_config_file,
            },
        )
        .robot_description_semantic(file_path='config/simplified_arm_assembly.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .to_moveit_configs()
    )

    # Get the path to the servo config file
    servo_config_file = os.path.join(
        get_package_share_directory('reseq_arm_mk2'), 'config', 'servo_config.yaml'
    )

    # Start move_group node (required for planning and IK)
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
        ],
    )
    launch_config.append(move_group_node)

    moveit_servo_exec = (
        'servo_node_main' if os.environ.get('ROS_DISTRO') == 'humble' else 'servo_node'
    )

    # Define the servo node, passing ALL required configs
    servo_node = Node(
        package='moveit_servo',
        executable=moveit_servo_exec,
        name='moveit_servo_node',
        parameters=[
            moveit_config.to_dict(),  # All MoveIt config
            servo_config_file,  # Servo config file (MUST BE LAST)
            {'use_sim_time': use_sim_time},  # Don't forget this for Gazebo
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'debug'],
    )
    launch_config.append(servo_node)

    moveit_controller_node = Node(
        package='reseq_arm_mk2',
        executable='moveit_controller.py',
        name='moveit_controller',
        parameters=[
            {
                'arm_module_address': 0,
                'use_sim_time': use_sim_time,
            }
        ],
        output='screen',
    )
    launch_config.append(moveit_controller_node)

    return launch_config


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time', default_value='false', choices=['true', 'false']
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
