from __future__ import annotations

import math
import traceback

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

# QoS profile for sensor topics (Gazebo bridge publishes BEST_EFFORT)
_SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)


def twist_magnitude(msg: Twist) -> float:
    return math.hypot(msg.linear.x, msg.angular.z)


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        self.output_topic = self.declare_parameter('output_topic', '/cmd_vel').value
        self.teleop_topic = self.declare_parameter('teleop_topic', '/cmd_vel_teleop').value
        self.nav_topic = self.declare_parameter('nav_topic', '/cmd_vel_nav').value
        self.autonomy_topic = self.declare_parameter('autonomy_topic', '/autonomy/enabled').value
        self.status_topic = self.declare_parameter('status_topic', '/autonomy/cmd_source').value
        self.health_topic = self.declare_parameter('health_topic', '/autonomy/health').value
        self.teleop_override_timeout = self.declare_parameter('teleop_override_timeout', 0.8).value
        self.nav_timeout = self.declare_parameter('nav_timeout', 0.8).value
        self.publish_rate = self.declare_parameter('publish_rate', 20.0).value
        self.manual_override_deadband = self.declare_parameter(
            'manual_override_deadband', 0.15
        ).value
        self.require_scan_for_autonomy = self.declare_parameter(
            'require_scan_for_autonomy', True
        ).value
        self.require_map_for_autonomy = self.declare_parameter(
            'require_map_for_autonomy', True
        ).value
        self.require_recent_map = self.declare_parameter('require_recent_map', True).value
        self.scan_timeout = self.declare_parameter('scan_timeout', 1.5).value
        self.map_timeout = self.declare_parameter('map_timeout', 3.0).value

        self.autonomy_enabled = False
        self.emergency_stop_active = False
        self.latest_teleop = Twist()
        self.latest_nav = Twist()
        self.last_teleop_time = None
        self.last_nav_time = None
        self.last_scan_time = None
        self.last_map_time = None

        self.output_pub = self.create_publisher(Twist, self.output_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.health_pub = self.create_publisher(String, self.health_topic, 10)

        self.create_subscription(Twist, self.teleop_topic, self.teleop_callback, 10)
        self.create_subscription(Twist, self.nav_topic, self.nav_callback, 10)
        self.create_subscription(Bool, self.autonomy_topic, self.autonomy_callback, 10)
        self.create_subscription(Bool, '/safety/estop', self.estop_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, _SENSOR_QOS)
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.create_timer(1.0 / self.publish_rate, self.publish_command)

    def teleop_callback(self, msg: Twist) -> None:
        self.latest_teleop = msg
        if twist_magnitude(msg) > self.manual_override_deadband:
            self.last_teleop_time = self.get_clock().now()

    def nav_callback(self, msg: Twist) -> None:
        self.latest_nav = msg
        self.last_nav_time = self.get_clock().now()

    def autonomy_callback(self, msg: Bool) -> None:
        self.autonomy_enabled = msg.data

    def estop_callback(self, msg: Bool) -> None:
        self.emergency_stop_active = msg.data

    def scan_callback(self, _msg: LaserScan) -> None:
        self.last_scan_time = self.get_clock().now()

    def map_callback(self, _msg: OccupancyGrid) -> None:
        self.last_map_time = self.get_clock().now()

    def is_recent(self, timestamp, timeout: float) -> bool:
        if timestamp is None:
            return False
        age = (self.get_clock().now() - timestamp).nanoseconds / 1e9
        return age <= timeout

    def autonomy_healthy(self) -> tuple[bool, str]:
        if self.require_scan_for_autonomy and not self.is_recent(
            self.last_scan_time, self.scan_timeout
        ):
            return False, 'waiting_for_scan'
        if self.require_map_for_autonomy:
            if self.last_map_time is None:
                return False, 'waiting_for_map'
            if self.require_recent_map and not self.is_recent(
                self.last_map_time, self.map_timeout
            ):
                return False, 'waiting_for_map'
        if not self.is_recent(self.last_nav_time, self.nav_timeout):
            return False, 'stale_nav_command'
        return True, 'ok'

    def publish_status(self, source: str, health: str) -> None:
        self.status_pub.publish(String(data=source))
        self.health_pub.publish(String(data=health))

    def publish_command(self) -> None:
        selected = Twist()
        source = 'teleop'
        health = 'manual_mode'

        teleop_override = self.is_recent(self.last_teleop_time, self.teleop_override_timeout)
        autonomy_ok, autonomy_health = self.autonomy_healthy()

        if self.emergency_stop_active:
            selected = Twist()
            source = 'safety_node'
            health = 'emergency_stop'
        elif not self.autonomy_enabled:
            selected = self.latest_teleop
        elif teleop_override:
            selected = self.latest_teleop
            source = 'teleop_override'
            health = 'manual_override'
        elif autonomy_ok:
            selected = self.latest_nav
            source = 'autonomy'
            health = 'autonomy_ok'
        else:
            source = 'safe_stop'
            health = autonomy_health

        self.output_pub.publish(selected)
        self.publish_status(source, health)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CmdVelMux()
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.logging.get_logger('cmd_vel_mux').warn('CmdVelMux interrupted by user')
    except Exception as err:
        rclpy.logging.get_logger('cmd_vel_mux').fatal(
            f'Error in the cmd_vel_mux node: {str(err)}\n{traceback.format_exc()}'
        )
    else:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
