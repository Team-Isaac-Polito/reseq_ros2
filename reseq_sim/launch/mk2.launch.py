import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    mk1_package_name = 'reseq_ros2'
    package_name = 'reseq_sim'

    # Read num_modules dynamically from the config
    description_share = get_package_share_directory('reseq_description')
    config_file = 'reseq_mk2_vcan.yaml'
    config_path = os.path.join(description_share, 'config', 'mk2', config_file)
    with open(config_path) as f:
        robot_config = yaml.safe_load(f)
    num_modules = str(robot_config.get('num_modules', 4))

    # Include the standard launch file which contain the robot_state_publisher node
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(mk1_package_name), 'launch', 'reseq_launch.py'
                )
            ]
        ),
        launch_arguments={
            'version': 'mk2',
            'autonomy': LaunchConfiguration('autonomy'),
            'arm': LaunchConfiguration('arm'),
            'use_sim_time': 'true',
            'use_ros2_control': 'true',
            'sim_mode': 'true',
            'sensors': 'false',
            'config_file': config_file,
            'map_file': LaunchConfiguration('map_file'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
        }.items(),
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name=package_name),
                    'launch',
                    'gazebo_launch.py',
                )
            ]
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'num_modules': num_modules,
        }.items(),
    )

    return [rsp, gazebo_launch]


def generate_launch_description():
    autonomy_arg = DeclareLaunchArgument(
        'autonomy', default_value='false', description='Enable autonomy in simulation'
    )
    arm_arg = DeclareLaunchArgument(
        'arm',
        default_value='true',
        description='Enable the arm stack in simulation',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_course.world',
        description=(
            'World to load in Gazebo. Available: simple_course.world, '
            'flat_krails.world, sloped_krails.world'
        ),
    )
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Static map yaml file that matches the simulation course (empty = use SLAM)',
    )
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='3.6',
        description='Initial x position for the robot spawn in Gazebo',
    )
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='3.6',
        description='Initial y position for the robot spawn in Gazebo',
    )
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.2',
        description='Initial z position for the robot spawn in Gazebo',
    )

    return LaunchDescription(
        [
            autonomy_arg,
            arm_arg,
            world_arg,
            map_file_arg,
            spawn_x_arg,
            spawn_y_arg,
            spawn_z_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
