import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

package_name = 'reseq_sim'


def launch_setup(context, *args, **kwargs):
    bridge_file = LaunchConfiguration('bridge_file').perform(context)

    # Ensure gz-sensors non-rendering plugins (IMU, altimeter, etc.) are discoverable.
    gz_sensors_lib = '/opt/ros/jazzy/opt/gz_sensors_vendor/lib'
    existing = os.environ.get('GZ_SENSORS_PLUGIN_PATH', '')
    if gz_sensors_lib not in existing:
        os.environ['GZ_SENSORS_PLUGIN_PATH'] = (
            gz_sensors_lib + ':' + existing if existing else gz_sensors_lib
        )

    ###################################
    # START GAZEBO AND SPAWN ENTITIES #
    ###################################

    # Launch gazebo with the world specified in `world_path`
    world_path = PathJoinSubstitution(
        [get_package_share_directory(package_name), 'worlds', LaunchConfiguration('world')]
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': ['-r -v4 ', world_path], 'on_exit_shutdown': 'true'}.items(),
    )

    # the `create` node is in charge of spawing the robot in gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',
            'reseq',
            '-topic',
            '/robot_description',
            '-x',
            LaunchConfiguration('spawn_x'),
            '-y',
            LaunchConfiguration('spawn_y'),
            '-z',
            LaunchConfiguration('spawn_z'),
        ],
        output='screen',
    )

    ##########
    # BRIDGE #
    ##########

    # Load base bridge config
    base_bridge_path = os.path.join(
        get_package_share_directory(package_name), 'config', bridge_file
    )
    with open(base_bridge_path) as f:
        bridge_config = yaml.safe_load(f)

    # Write merged config to a temp file so the bridge node can read it
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, prefix='gz_bridge_merged_'
    )
    yaml.dump(bridge_config, tmp)
    tmp.close()

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': tmp.name}],
    )

    return [gazebo, spawn_entity, ros_gz_bridge]


def generate_launch_description():
    ####################
    # LAUNCH ARGUMENTS #
    ####################

    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.world', description='World to load in the gazebo simulation'
    )

    bridge_file_arg = DeclareLaunchArgument(
        'bridge_file',
        default_value='gz_bridge.yaml',
        description='File containing bridge topics between gazebo and ros2',
    )

    num_modules_arg = DeclareLaunchArgument(
        'num_modules',
        default_value='4',
        description='Number of robot modules — used to add IMU bridge entries dynamically',
    )

    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='Initial x position for the robot spawn in Gazebo',
    )

    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Initial y position for the robot spawn in Gazebo',
    )

    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.2',
        description='Initial z position for the robot spawn in Gazebo',
    )

    return LaunchDescription(
        [
            world_arg,
            bridge_file_arg,
            num_modules_arg,
            spawn_x_arg,
            spawn_y_arg,
            spawn_z_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
