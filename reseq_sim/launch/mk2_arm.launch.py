import os
import subprocess
import time

import rclpy
import xacro
from ament_index_python.packages import get_package_share_directory
from controller_manager import (
    configure_controller,
    list_controllers,
    load_controller,
    switch_controllers,
)
from controller_manager.controller_manager_services import ServiceNotFoundError
from controller_manager_msgs.srv import SwitchController
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _wait_for_controllers(node, timeout_sec: float = 60.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        try:
            return list_controllers(
                node,
                '/controller_manager',
                service_timeout=1.0,
                call_timeout=10.0,
            ).controller
        except ServiceNotFoundError:
            time.sleep(1.0)

    raise RuntimeError('Timed out waiting for /controller_manager/list_controllers')

def _ensure_controller_active(node, controller_name: str):
    controllers = _wait_for_controllers(node)
    controller = next((c for c in controllers if c.name == controller_name), None)

    if controller is None:
        ret = load_controller(
            node,
            '/controller_manager',
            controller_name,
            service_timeout=10.0,
            call_timeout=10.0,
        )
        if not ret.ok:
            raise RuntimeError(f'Failed to load controller {controller_name}')
        controller_state = 'unconfigured'
    else:
        controller_state = controller.state

    if controller_state == 'active':
        node.get_logger().info(f'Controller {controller_name} already active.')
        return

    if controller_state == 'unconfigured':
        ret = configure_controller(
            node,
            '/controller_manager',
            controller_name,
            service_timeout=10.0,
            call_timeout=10.0,
        )
        if not ret.ok:
            raise RuntimeError(f'Failed to configure controller {controller_name}')
        controller_state = 'inactive'

    if controller_state == 'inactive':
        ret = switch_controllers(
            node,
            '/controller_manager',
            [],
            [controller_name],
            SwitchController.Request.STRICT,
            True,
            10.0,
            10.0,
        )
        if not ret.ok:
            raise RuntimeError(f'Failed to activate controller {controller_name}')

    node.get_logger().info(f'Controller {controller_name} active.')


def ensure_sim_controllers(context, *args, **kwargs):
    launch_joint_state_broadcaster = (
        LaunchConfiguration('launch_joint_state_broadcaster').perform(context).lower() == 'true'
    )
    launch_cartesian_controller = (
        LaunchConfiguration('launch_cartesian_controller').perform(context).lower() == 'true'
    )

    controller_names = []
    if launch_joint_state_broadcaster:
        controller_names.append('joint_state_broadcaster')
    if launch_cartesian_controller:
        controller_names.append('joint_group_velocity_controller')
    else:
        controller_names.append('mk2_arm_controller')

    shutdown_rclpy = False
    if not rclpy.ok():
        rclpy.init(args=None)
        shutdown_rclpy = True

    node = rclpy.create_node('arm_controller_setup')
    try:
        for controller_name in controller_names:
            _ensure_controller_active(node, controller_name)
    finally:
        node.destroy_node()
        if shutdown_rclpy and rclpy.ok():
            rclpy.shutdown()

    return []


def launch_setup(context, *args, **kwargs):
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    use_moveit = LaunchConfiguration('use_moveit').perform(context).lower() == 'true'
    launch_rsp = LaunchConfiguration('launch_rsp').perform(context).lower() == 'true'
    launch_cartesian_controller = (
        LaunchConfiguration('launch_cartesian_controller').perform(context).lower() == 'true'
    )
    arm_max_cartesian_vel = float(LaunchConfiguration('arm_max_cartesian_vel').perform(context))
    arm_max_joint_vel = float(LaunchConfiguration('arm_max_joint_vel').perform(context))
    arm_trajectory_horizon_sec = float(
        LaunchConfiguration('arm_trajectory_horizon_sec').perform(context)
    )
    arm_jacobian_damping = float(LaunchConfiguration('arm_jacobian_damping').perform(context))

    package_name = 'reseq_sim'
    description_share = get_package_share_directory('reseq_description')
    robot_config_file = os.path.join(description_share, 'config', 'mk2', 'reseq_mk2_vcan.yaml')

    generate_configs = subprocess.run(
        [
            'python3',
            os.path.join(description_share, 'scripts/generate_configs.py'),
            'reseq_mk2_vcan.yaml',
            '--version',
            'mk2',
            '--no_body_controllers',
        ]
        + (['--use_sim_time'] if sim else []),
        check=True,
        capture_output=True,
        text=True,
    )

    if generate_configs.stdout:
        print(generate_configs.stdout)
    if generate_configs.stderr:
        print(generate_configs.stderr)

    xacro_file = get_package_share_directory('reseq_arm_mk2') + '/urdf/arm.urdf.xacro'

    controllers_config_file = os.path.join(
        description_share,
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
        launch_entities.append(
            Node(
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
        )

    launch_entities.append(
        IncludeLaunchDescription(
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
            }.items(),
        )
    )

    launch_entities.append(
        TimerAction(
            period=0.5,
            actions=[OpaqueFunction(function=ensure_sim_controllers)],
        )
    )

    if launch_cartesian_controller:
        launch_entities.append(
            Node(
                package='reseq_arm_mk2',
                executable='cartesian_arm_controller',
                name='cartesian_arm_controller',
                parameters=[
                    {'robot_description': robot_description},
                    {'use_sim_time': True},
                    {'state_topic': '/joint_states'},
                    {'chain_tip': 'tcp'},
                    {'command_mode': 'velocity'},
                    {'command_frame': 'arm_base_link'},
                    {'max_cartesian_vel': arm_max_cartesian_vel},
                    {'max_joint_vel': arm_max_joint_vel},
                    {'startup_hold_positions': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                    {'deadzone': 0.02},
                    {'trajectory_horizon_sec': arm_trajectory_horizon_sec},
                    {'jacobian_damping': arm_jacobian_damping},
                ],
                output='screen',
            )
        )
    else:
        launch_entities.append(
            Node(
                package='reseq_arm_mk2',
                executable='arm_state_bridge',
                name='arm_state_bridge',
                parameters=[
                    {
                        'source_topic': '/joint_states',
                        'output_mode': 'trajectory',
                        'trajectory_topic': '/mk2_arm_controller/joint_trajectory',
                        'trajectory_duration_sec': 0.1,
                        'startup_positions': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    }
                ],
                output='screen',
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

    return launch_entities


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value='empty.world',
                description='World to load for the Gazebo arm simulation',
            ),
            DeclareLaunchArgument(
                'sim',
                default_value='true',
                description='Use the Gazebo simulation controllers',
            ),
            DeclareLaunchArgument(
                'use_moveit',
                default_value='false',
                description='Launch MoveIt-related arm tools',
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
                default_value='true',
                description=(
                    'Launch the local Cartesian arm controller instead of '
                    'following the Jetson trajectory bridge'
                ),
            ),
            DeclareLaunchArgument(
                'arm_max_cartesian_vel',
                default_value='0.4',
                description='Cartesian velocity scale for the Gazebo arm controller',
            ),
            DeclareLaunchArgument(
                'arm_max_joint_vel',
                default_value='0.8',
                description='Joint velocity clamp for the Gazebo arm controller',
            ),
            DeclareLaunchArgument(
                'arm_trajectory_horizon_sec',
                default_value='0.1',
                description='Trajectory lookahead horizon for the Gazebo arm controller',
            ),
            DeclareLaunchArgument(
                'arm_jacobian_damping',
                default_value='0.04',
                description='Jacobian damping term for the Gazebo arm controller',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
