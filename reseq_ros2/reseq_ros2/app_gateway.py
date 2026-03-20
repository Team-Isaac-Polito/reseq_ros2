import os
import signal
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Empty
from reseq_interfaces.srv import NodeStatus

class AppGateway(Node):
    def __init__(self):
        super().__init__('app_gateway')

        self.service_mapping = {
            'thermal': 'activate_thermal',
            'lidar': 'rplidar_mode/toggle_scan',
            'velocity': 'vel'
        }
        
        self.node_launchers = {
            'thermal': ('thermal_node', ['ros2', 'run', 'reseq_ros2', 'thermal_node']),
            'lidar': ('rplidar_mode', ['ros2', 'run', 'reseq_ros2', 'rplidar_mode']),
            'velocity': ('velocity', ['ros2', 'run', 'reseq_ros2', 'velocity'])
        }
        
        self.hw_clients = {}
        self.hw_processes = {}  # Track running processes
        self.node_states = {'thermal': False, 'lidar': False, 'velocity': False}
        
        for ui_name, hw_service in self.service_mapping.items():
            ui_service_name = f'/ui/{ui_name}'
            self.create_service(
                NodeStatus,
                ui_service_name,
                lambda req, res, n=ui_name: self.universal_callback(req, res, n)
            )
            self.hw_clients[ui_name] = self.create_client(SetBool, hw_service)

        # Service to query node status
        self.create_service(NodeStatus, '/ui/node_status', self.handle_node_status)
        self.get_logger().info('Gateway App initialized. Shutdown hook for LiDAR active.')

    def stop_rplidar_on_shutdown(self):
        """
        Specialized function to call the hardware /stop_motor service.
        This is called during the Gateway shutdown to ensure the motor halts.
        """
        self.get_logger().info('[Shutdown Hook] Attempting to stop LiDAR motor...')
        
        # We create a specific client for the hardware level /stop_motor
        stop_client = self.create_client(Empty, '/stop_motor')
        
        if stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('[Shutdown Hook] Service /stop_motor found. Sending signal...')
            stop_client.call_async(Empty.Request())
            # Give it a moment to reach the hardware before the node is destroyed
            time.sleep(1.0) 
        else:
            self.get_logger().warn('[Shutdown Hook] /stop_motor not available. Lidar might already be off.')

    def _stop_node(self, module_id):
        """Standard node termination logic."""
        if module_id not in self.hw_processes:
            return True, 'Node not running'
        
        process = self.hw_processes[module_id]
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=2.0)
            del self.hw_processes[module_id]
            self.node_states[module_id] = False
            return True, 'Process terminated'
        except Exception as e:
            return False, str(e)

    def universal_callback(self, request, response, module_id):
        action = request.status.lower()
        hw_request = SetBool.Request()

        if action == 'query':
            state = 'RUNNING' if self.node_states.get(module_id) else 'STOPPED'
            response.success = True; response.message = f'{module_id}: {state}'
            return response

        if action == 'enable':
            success, msg = self._ensure_node_running(module_id)
            if not success:
                response.success = False; response.message = msg
                return response
            hw_request.data = True
        elif action == 'disable':
            hw_request.data = False
        
        client = self.hw_clients[module_id]
        if client.wait_for_service(timeout_sec=2.0):
            client.call_async(hw_request)
            if action == 'disable':
                time.sleep(0.5)
                self._stop_node(module_id)
            response.success = True
            response.message = f'{module_id} command successful'
        else:
            response.success = False
            response.message = 'Service timeout'
        return response

    def handle_node_status(self, request, response):
        available_services = [s[0] for s in self.get_service_names_and_types()]
        status_lines = []
        for module_id, hw_service in self.service_mapping.items():
            is_running = any(hw_service in s for s in available_services)
            status_lines.append(f"{module_id}: {'RUNNING' if is_running else 'STOPPED'}")
        response.message = '\n'.join(status_lines)
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AppGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt: Initiating shutdown sequence...')
    finally:
        # EXECUTE SHUTDOWN SERVICE CALL BEFORE DESTROYING NODE
        node.stop_rplidar_on_shutdown()
        
        # Cleanup any other running subprocesses
        for mid in list(node.hw_processes.keys()):
            node._stop_node(mid)
            
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()