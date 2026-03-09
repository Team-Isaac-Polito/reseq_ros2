import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

from reseq_interfaces.srv import NodeStatus


class AppGateway(Node):
    def __init__(self):
        super().__init__('app_gateway')

        self.service_mapping = {
            'thermal': 'activate_thermal',
            'lidar': 'rplidar_mode/toggle_scan',
            'velocity': 'vel',
        }

        # Node launchers: mapping to how to launch each node
        self.node_launchers = {
            'thermal': ('thermal_node', ['ros2', 'run', 'reseq_ros2', 'thermal_node']),
            'lidar': ('rplidar_mode', ['ros2', 'run', 'reseq_ros2', 'rplidar_mode']),
            'velocity': ('velocity', ['ros2', 'run', 'reseq_ros2', 'velocity']),
        }

        self.hw_clients = {}
        self.hw_processes = {}  # Track running processes
        self.node_states = {
            'thermal': False,  # started or not
            'lidar': False,
            'velocity': False,
        }

        for ui_name, hw_service in self.service_mapping.items():
            # service names on the module card of flutter app
            ui_service_name = f'/ui/{ui_name}'
            self.create_service(
                NodeStatus,
                ui_service_name,
                lambda req, res, n=ui_name: self.universal_callback(req, res, n),
            )
            self.hw_clients[ui_name] = self.create_client(SetBool, hw_service)
            self.get_logger().info(f'Mapped: {ui_service_name} -> {hw_service}')

        # Service to query node status
        self.create_service(NodeStatus, '/ui/node_status', self.handle_node_status)
        self.get_logger().info('Service /ui/node_status available for node status queries')
        self.get_logger().info('Gateway App initialized correctly (lazy-loading mode).')

    def _ensure_node_running(self, module_id):
        """
        Ensure a node is running. Launch it if not already running.
        Returns: (success: bool, message: str)
        """
        if module_id not in self.node_launchers:
            return False, f'Module {module_id} not configured'

        node_name, launch_cmd = self.node_launchers[module_id]

        # Check if process is still running
        if module_id in self.hw_processes and self.hw_processes[module_id] is not None:
            process = self.hw_processes[module_id]
            if process.poll() is None:  # Process still alive
                self.get_logger().info(f'Node {node_name} already running (PID: {process.pid})')
                return True, f'{node_name} already active'
            else:
                # Process died, remove it
                del self.hw_processes[module_id]
                self.node_states[module_id] = False

        # Launch the node
        try:
            self.get_logger().info(f'Starting node on-demand: {node_name}')
            process = subprocess.Popen(
                launch_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid
            )
            self.hw_processes[module_id] = process
            self.node_states[module_id] = True
            self.get_logger().info(f'Node {node_name} started (PID: {process.pid})')

            # Give node time to initialize and register its service (2 seconds for safety)
            time.sleep(2.0)

            return True, f'{node_name} started successfully'
        except Exception as e:
            self.get_logger().error(f'Error starting {node_name}: {e}')
            self.node_states[module_id] = False
            return False, f'Error starting: {str(e)}'

    def _stop_node(self, module_id):
        """
        Stop a running node.
        Returns: (success: bool, message: str)
        """
        if module_id not in self.node_launchers:
            return False, f'Module {module_id} not configured'

        node_name, _ = self.node_launchers[module_id]

        if module_id not in self.hw_processes or self.hw_processes[module_id] is None:
            self.node_states[module_id] = False
            return True, f'{node_name} already stopped'

        process = self.hw_processes[module_id]
        if process.poll() is not None:  # Already dead
            self.node_states[module_id] = False
            del self.hw_processes[module_id]
            return True, f'{node_name} was already stopped'

        try:
            self.get_logger().info(f'Stopping node: {node_name} (PID: {process.pid})')
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)

            try:
                process.wait(timeout=3.0)
                self.get_logger().info(f'Nodo {node_name} fermato con SIGTERM')
            except subprocess.TimeoutExpired:
                self.get_logger().warning(f'SIGTERM timeout for {node_name}, sending SIGKILL')
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait(timeout=2.0)
                self.get_logger().info(f'Node {node_name} stopped with SIGKILL')

            self.node_states[module_id] = False
            del self.hw_processes[module_id]
            return True, f'{node_name} stopped successfully'

        except Exception as e:
            self.get_logger().error(f'Error stopping {node_name}: {e}')
            self.node_states[module_id] = False
            return False, f'Error stopping: {str(e)}'

    def _cleanup_processes(self):
        """Clean up all launched processes on shutdown"""
        for module_id in list(self.hw_processes.keys()):
            self._stop_node(module_id)

    def universal_callback(self, request, response, module_id):
        """
        Universal callback that handles UI requests.
        If the node is not active, it starts it on demand.
        Then forwards the request to the hardware node.
        """
        action = request.status.lower()
        self.get_logger().info(f'UI request for module [{module_id}]: {action}')

        if action == 'enable':
            success, msg = self._ensure_node_running(module_id)
            if not success:
                response.success = False
                response.message = f'Error starting: {msg}'
                return response

            hw_request = SetBool.Request()
            hw_request.data = True
        else:
            success, msg = self._stop_node(module_id)
            response.success = success
            response.message = msg
            return response
        client = self.hw_clients[module_id]
        if not client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = 'Node started but hardware service not responding'
            return response
        client.call_async(hw_request)
        response.success = True
        response.message = f'{module_id} activated successfully'
        return response

    def handle_node_status(self, request, response):
        """
        Handler for the /ui/node_status service.
        Returns the status of all managed nodes.
        """
        self.get_logger().info('Node status query received')

        status_lines = []
        for module_id in ['thermal', 'lidar', 'velocity']:
            if module_id in self.hw_processes and self.hw_processes[module_id] is not None:
                process = self.hw_processes[module_id]
                is_running = process.poll() is None
                self.node_states[module_id] = is_running

            state = 'RUNNING' if self.node_states.get(module_id, False) else 'STOPPED'
            status_lines.append(f'{module_id}: {state}')

        response.message = '\n'.join(status_lines)
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AppGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cleanup_processes()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
