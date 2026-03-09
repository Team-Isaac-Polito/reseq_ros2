import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class RplidarNode(Node):
    """Node that manages rplidar start/stop via SetBool service."""

    # Launch command for rplidar - customize based on your hardware
    # Common options: rplidar_a2m8_launch.py, rplidar_s1_launch.py, etc.
    LAUNCH_CMD = ['ros2', 'launch', 'rplidar_ros', 'rplidar_a2m8_launch.py']

    def __init__(self):
        super().__init__('rplidar_mode')
        self.enable = False
        self.process = None

        # Create the ROS2 service
        self.srv = self.create_service(SetBool, 'rplidar_mode/toggle_scan', self.handle_toggle)
        self.get_logger().info('RPLidar node ready. Service /rplidar_mode/toggle_scan active.')

    def handle_toggle(self, request, response):
        """Callback for the ROS2 service."""
        self.enable = request.data

        if self.enable:
            response.success, response.message = self.start_rplidar()
        else:
            response.success, response.message = self.stop_rplidar()

        return response

    def start_rplidar(self):
        """Start the rplidar process."""
        if self.process is not None and self.process.poll() is None:
            return True, 'RPLidar already running.'

        try:
            self.get_logger().info(f'Starting RPLidar: {" ".join(self.LAUNCH_CMD)}')
            self.process = subprocess.Popen(
                self.LAUNCH_CMD,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )

            # Wait briefly to verify the process started
            time.sleep(0.5)
            if self.process.poll() is not None:
                stderr = self.process.stderr.read().decode('utf-8', errors='ignore')
                return False, f'RPLidar did not start: {stderr[:200]}'

            self.get_logger().info('RPLidar started successfully.')
            return True, 'RPLidar started.'

        except Exception as e:
            self.get_logger().error(f'Error starting RPLidar: {e}')
            return False, f'Error: {str(e)}'

    def stop_rplidar(self):
        """Stop the rplidar process."""
        if self.process is None or self.process.poll() is not None:
            return True, 'RPLidar already stopped.'

        try:
            self.get_logger().info('Stopping RPLidar...')
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)

            try:
                self.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warning('RPLidar did not stop, sending SIGKILL...')
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                self.process.wait(timeout=2.0)

            self.get_logger().info('RPLidar stopped.')
            self.process = None
            return True, 'RPLidar stopped.'

        except Exception as e:
            self.get_logger().error(f'Error stopping RPLidar: {e}')
            return False, f'Error: {str(e)}'

    def switchRplidar(self):
        """Legacy method - toggle manuale."""
        self.enable = not self.enable
        if self.enable:
            self.start_rplidar()
        else:
            self.stop_rplidar()


def main(args=None):
    """Entry point for the node."""
    rclpy.init(args=args)
    node = RplidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup: stop rplidar if running
        if node.process is not None and node.process.poll() is None:
            try:
                os.killpg(os.getpgid(node.process.pid), signal.SIGTERM)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
