from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.events import Shutdown
from launch_param_builder import ParameterBuilder

from moveit_configs_utils import MoveItConfigsBuilder
import os

from reseq_ros2.utils.launch_utils import (
    config_path,
    default_filename,
)

share_folder = get_package_share_directory('reseq_ros2')

# launch_setup is used through an OpaqueFunction because it is the only way to manipulate a
# command line argument directly in the launch file
def launch_setup(context, *args, **kwargs):
    # Get config path from command line, otherwise use the default path
    log_level = LaunchConfiguration('log_level').perform(context)
    digital_twin_enabled = LaunchConfiguration('d_twin').perform(context)
    external_log_level = LaunchConfiguration('external_log_level').perform(context)
   
    launch_config = []

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="reseq", 
            package_name="reseq_ros2"
        )
        .robot_description(file_path="config/simplified_arm_assembly.urdf.xacro")
        .robot_description_semantic(file_path="config/simplified_arm_assembly.srdf")
        .robot_description_kinematics(file_path="config/bio_ik_kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )

    servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml(os.path.join(
            get_package_share_directory("reseq_ros2"),
            "config",
            "servo_config.yaml"
        ))
        .to_dict()    
    }

    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "mk2_arm"}

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

    mk2_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mk2_arm_controller", "-c", "/controller_manager"],
    )
    launch_config.append(mk2_arm_controller_spawner)

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits
        ],
        arguments=['--ros-args', '--log-level', external_log_level],
        output="screen",
    )
    launch_config.append(servo_node)

    moveit_controller_node = Node(
        package='reseq_ros2',
        executable='moveit_controller',
        name='moveit_controller',
        #parameters=[],
        arguments=['--ros-args', '--log-level', log_level],
        on_exit=Shutdown(),
    )
    launch_config.append(moveit_controller_node)

    if digital_twin_enabled == 'false':
        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="log",
            parameters=[moveit_config.robot_description],
        )
        launch_config.append(robot_state_publisher_node)

    return launch_config


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('log_level', default_value='info'),
            DeclareLaunchArgument('external_log_level', default_value='warn'),
            DeclareLaunchArgument('d_twin', default_value='true'),
            OpaqueFunction(function=launch_setup),
        ]
    )
