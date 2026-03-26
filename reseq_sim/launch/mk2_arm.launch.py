import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    use_moveit = LaunchConfiguration('use_moveit').perform(context).lower() == 'true'
    launch_rsp = LaunchConfiguration('launch_rsp').perform(context).lower() == 'true'
    launch_joint_state_broadcaster = (
        LaunchConfiguration('launch_joint_state_broadcaster').perform(context).lower() == 'true'
    )
    launch_cartesian_controller = (
        LaunchConfiguration('launch_cartesian_controller').perform(context).lower() == 'true'
    )

    package_name = 'reseq_sim'
    description_share = get_package_share_directory('reseq_description')
    robot_config_file = os.path.join(description_share, 'config', 'mk2', 'reseq_mk2_vcan.yaml')

    generate_configs = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(description_share, 'scripts/generate_configs.py'),
            'reseq_mk2_vcan.yaml',
            '--version',
            'mk2',
            '--no_body_controllers',
        ]
        + (['--use_sim_time'] if sim else []),
        name='generate_configs',
        output='screen',
    )

    xacro_file = get_package_share_directory('reseq_arm_mk2') + '/urdf/arm.urdf.xacro'

    controllers_config_file = os.path.join(
        get_package_share_directory('reseq_description'),
        'config',
        'temp',
        'reseq_controllers.yaml',
    )
    print(f'Using controllers config file: {controllers_config_file}')

    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'sim_mode': 'true' if sim else 'false',
            'version': 'mk2',
            'config_path': robot_config_file,
            'controllers_config_file': controllers_config_file,
        },
    ).toxml()

    launch_entities = []

    if launch_rsp:
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description,
                    'use_sim_time': True,
                }
            ],
        )
        launch_entities.append(rsp)

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name=package_name),
                    'launch',
                    'gazebo_launch.py',
                )
            ]
        )
    )

    launch_entities.append(gazebo_launch)

    controller_manager_ready = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            (
                'until ros2 service list --include-hidden-services '
                '| grep -Fxq /controller_manager/list_controllers; '
                'do sleep 1; done'
            ),
        ],
        output='screen',
    )

    body_spawners = []
    if launch_joint_state_broadcaster:
        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager',
                '/controller_manager',
            ],
        )
        body_spawners.append(joint_state_broadcaster_spawner)
    else:
        joint_state_broadcaster_spawner = None

    if launch_cartesian_controller:
        # Launch the same Cartesian controller path as the real arm so Gazebo
        # follows the same joint-trajectory commands.
        cartesian_arm_controller_node = Node(
            package='reseq_arm_mk2',
            executable='cartesian_arm_controller',
            name='cartesian_arm_controller',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': True},
                {'chain_tip': 'tool0'},
                {'command_mode': 'trajectory'},
                {'command_frame': 'arm_base_link'},
                {'max_cartesian_vel': 0.6},
                {'max_joint_vel': 1.0},
                {'deadzone': 0.02},
                {'trajectory_horizon_sec': 0.1},
            ],
            output='screen',
        )
        launch_entities.append(cartesian_arm_controller_node)
    else:
        arm_state_bridge_node = Node(
            package='reseq_arm_mk2',
            executable='arm_state_bridge',
            name='arm_state_bridge',
            parameters=[
                {
                    'source_topic': '/arm_joint_states',
                    'output_mode': 'trajectory',
                    'trajectory_topic': '/mk2_arm_controller/joint_trajectory',
                    'trajectory_duration_sec': 0.1,
                }
            ],
            output='screen',
        )
        launch_entities.append(arm_state_bridge_node)

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mk2_arm_controller', '--controller-manager', '/controller_manager'],
    )

    launch_entities.append(controller_manager_ready)

    if launch_joint_state_broadcaster:
        launch_entities.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=controller_manager_ready,
                    on_exit=[body_spawners[0]],
                )
            )
        )
        launch_entities.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=body_spawners[0],
                    on_exit=[arm_controller_spawner],
                )
            )
        )
    else:
        launch_entities.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=controller_manager_ready,
                    on_exit=[arm_controller_spawner],
                )
            )
        )

    if use_moveit:
        launch_entities.append(
            Node(
                package='reseq_arm_mk2',
                executable='coordinate_controller',
                name='coordinate_controller',
                parameters=[{'robot_description': robot_description}],
                output='screen',
            )
        )

    return [generate_configs, *launch_entities]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('ros_domain_id', default_value='42'),
            DeclareLaunchArgument(
                'sim', default_value='true', description='Use the Gazebo simulation controllers'
            ),
            SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
            SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
            SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
            DeclareLaunchArgument(
                'use_moveit',
                default_value='true',
                description='Launch MoveIt (for IK / RViz visualisation)',
            ),
            DeclareLaunchArgument(
                'launch_rsp',
                default_value='true',
                description='Launch robot_state_publisher for the Gazebo stack',
            ),
            DeclareLaunchArgument(
                'launch_joint_state_broadcaster',
                default_value='true',
                description='Launch joint_state_broadcaster for the Gazebo stack',
            ),
            DeclareLaunchArgument(
                'launch_cartesian_controller',
                default_value='false',
                description=(
                    'Launch the local Cartesian arm controller instead of '
                    'following the Jetson trajectory bridge'
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
