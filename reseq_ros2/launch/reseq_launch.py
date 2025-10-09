import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from reseq_ros2.utils.launch_utils import default_filename

share_folder = get_package_share_directory('reseq_ros2')


def launch_setup(context, *args, **kwargs):
    config_filename = LaunchConfiguration('config_file').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    external_log_level = LaunchConfiguration('external_log_level').perform(context)
    launch_config = []

    sensors_enabled = LaunchConfiguration('sensors').perform(context)
    digital_twin_enabled = LaunchConfiguration('d_twin').perform(context)
    use_webcam = LaunchConfiguration('use_webcam').perform(context)

    # Core launch file
    core_launch_file = os.path.join(
        get_package_share_directory('reseq_ros2'), 'launch', 'reseq_core_launch.py'
    )
    launch_config.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch_file),
            launch_arguments={
                'config_file': config_filename,
                'log_level': log_level,
                'use_webcam': use_webcam,
            }.items(),
        )
    )

    # Sensor launch file (for robot)
    if sensors_enabled == 'true' and use_webcam == 'false':
        sensors_launch_file = os.path.join(
            get_package_share_directory('reseq_ros2'), 'launch', 'sensors_launch.py'
        )
        launch_config.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensors_launch_file),
                launch_arguments={
                    'config_file': config_filename,
                    'external_log_level': external_log_level,
                }.items(),
            )
        )

    if use_webcam == 'true':
        webcam_launch_file = os.path.join(
            get_package_share_directory('reseq_ros2'), 'launch', 'webcam_launch.py'
        )
        launch_config.append(
            IncludeLaunchDescription(PythonLaunchDescriptionSource(webcam_launch_file))
        )

    # Digital twin (for robot)
    if digital_twin_enabled == 'true' and use_webcam == 'false':
        digital_twin_launch_file = os.path.join(
            get_package_share_directory('reseq_ros2'), 'launch', 'digital_twin_launch.py'
        )
        launch_config.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(digital_twin_launch_file),
                launch_arguments={
                    'config_file': config_filename,
                    'log_level': log_level,
                    'external_log_level': external_log_level,
                }.items(),
            )
        )

    # Apriltag launch file
    apriltag_launch_file = os.path.join(
        get_package_share_directory('reseq_ros2'), 'launch', 'apriltag_launch.py'
    )
    launch_config.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(apriltag_launch_file),
            launch_arguments={
                'use_webcam': use_webcam,
            }.items(),
        )
    )

    # Start detector node and set the correct image topic
    image_topic = (
        '/webcam/image_raw'
        if use_webcam == 'true'
        else '/realsense/realsense2_camera_node/color/image_raw'
    )

    detector_node = Node(
        package='reseq_ros2',
        executable='detector',
        name='detector',
        parameters=[{'image_topic': image_topic}],
        output='screen',
    )
    launch_config.append(detector_node)

    # We need a robot_state_publisher for the apriltag TFs to work
    if use_webcam == 'true':
        urdf_content = """
        <?xml version="1.0"?>
        <robot name="minimal_robot">
          <link name="base_link"/>
          <link name="camera_link"/>
          <joint name="camera_joint" type="fixed">
            <parent link="base_link"/>
            <child link="camera_link"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
          </joint>
        </robot>
        """
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': urdf_content}],
        )
        launch_config.append(robot_state_publisher_node)

    return launch_config


def generate_launch_description():
    generate_configs = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(share_folder, 'scripts/generate_configs.py'),
            LaunchConfiguration('config_file'),
        ],
        name='generate_configs',
        output='screen',
    )

    on_generate_configs_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=generate_configs,
            on_exit=[
                LogInfo(msg='Configuration files generated.'),
                OpaqueFunction(function=launch_setup),
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('config_file', default_value=default_filename),
            DeclareLaunchArgument('sensors', default_value='true', description='Enable sensors'),
            DeclareLaunchArgument(
                'use_webcam',
                default_value='false',
                description='Use webcam instead of robot sensors',
            ),
            DeclareLaunchArgument(
                'd_twin', default_value='true', description='Enable digital twin'
            ),
            DeclareLaunchArgument(
                'log_level', default_value='info', description='Set log level for reseq nodes'
            ),
            DeclareLaunchArgument(
                'external_log_level',
                default_value='warn',
                description='Set log level for external nodes',
            ),
            generate_configs,
            on_generate_configs_exit,
        ]
    )
