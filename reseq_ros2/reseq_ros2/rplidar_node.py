import os
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, SetBool
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class RplidarNode(Node):
    """Node that manages rplidar start/stop via SetBool service."""
    
    LAUNCH_CMD = ['ros2', 'launch', 'rplidar_ros', 'rplidar_a2m8_launch.py']

    def __init__(self):
        super().__init__('rplidar_mode')
        self.process = None
        self.enable = False
        self.callback_group = ReentrantCallbackGroup()

        self.srv = self.create_service(
            SetBool,
            'rplidar_mode/toggle_scan',
            self.handle_toggle,
            callback_group=self.callback_group
        )

        self.start_motor_cli = self.create_client(Empty, 'start_motor', callback_group=self.callback_group)
        self.stop_motor_cli = self.create_client(Empty, 'stop_motor', callback_group=self.callback_group)
        self.get_logger().info('Lidar wrapper initialized')
    
    def handle_toggle(self, request, response):
        """Callback per il servizio ROS2."""
        self.enable = request.data
        
        if self.enable:
            response.success, response.message = self.start_rplidar()
        else:
            response.success, response.message = self.stop_rplidar()
        
        return response
    
    def start_rplidar(self):
        """Start the rplidar process."""
        if self.process is not None and self.process.poll() is None:
            return True, 'Rplidar is running'

        try:
            self.get_logger().info("Starting RPLidar driver via subprocess...")
            # self.process = subprocess.Popen(
            #     self.LAUNCH_CMD,
            #     stdout=subprocess.PIPE,
            #     stderr=subprocess.PIPE,
            #     preexec_fn=os.setsid
            # )

            self.get_logger().info("Waiting for hardware services to activate...")
            time.sleep(3.0) 

            if self.start_motor_cli.wait_for_service(timeout_sec=5.0):
                self.start_motor_cli.call_async(Empty.Request())
                return True, "Driver started and motor rotation activated."
            else:
                return True, "Driver started, but motor services not responding (timeout)."

        except Exception as e:
            self.get_logger().error(f"Critical startup error: {str(e)}")
            return False, f"Error starting process: {str(e)}"

    
    def stop_rplidar(self):
        """Stop the motor and close the driver."""
        if self.process is None:
            return True, 'RPLidar was not active.'

        try:
            if self.stop_motor_cli.wait_for_service(timeout_sec=1.0):
                self.stop_motor_cli.call_async(Empty.Request())
                time.sleep(0.5)

            self.get_logger().info('Stopping hardware driver...')
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=2.0)
            self.process = None
            return True, "RPLidar stopped correctly."
            
        except Exception as e:
            return False, f"Error during shutdown: {str(e)}"


def main(args=None):
    """Entry point for the node."""
    rclpy.init(args=args)
    node = RplidarNode()
    executor = MultiThreadedExecutor()
    executor.add_node(executor)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup: ferma rplidar se in esecuzione
        if node.process is not None and node.process.poll() is None:
            try:
                os.killpg(os.getpgid(node.process.pid), signal.SIGTERM)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()