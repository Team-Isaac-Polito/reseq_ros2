import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import subprocess
import os
import signal
import time

class AppGateway(Node):
    def __init__(self):
        super().__init__('app_gateway')

        self.service_mapping = {
            'thermal': 'activate_thermal',
            'lidar': 'rplidar_mode/toggle_scan'
        }
        
        self.node_launchers = {
            'thermal': ('thermal_node', ['ros2', 'run', 'reseq_ros2', 'thermal_node']),
            'lidar': ('rplidar_mode', ['ros2', 'run', 'reseq_ros2', 'rplidar_mode']),
        }
        
        self.hw_clients = {}
        self.hw_processes = {}
        
        self._launch_dependent_nodes()
        
        for ui_name, hw_service in self.service_mapping.items():
            ui_service_name = f'ui/{ui_name}'
            self.create_service(SetBool, ui_service_name, 
                               lambda req, res, n=ui_name: self.universal_callback(req, res, n))
            self.hw_clients[ui_name] = self.create_client(SetBool, hw_service)
            self.get_logger().info(f"Mappato: {ui_service_name} -> {hw_service}")

        self.get_logger().info("Gateway App initialized correctly.")
    
    def _launch_dependent_nodes(self):
        """Launch all dependent hardware nodes"""
        for hw_name, (node_name, launch_cmd) in self.node_launchers.items():
            try:
                self.get_logger().info(f"Avvio nodo dipendente: {node_name}")
                process = subprocess.Popen(
                    launch_cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid
                )
                self.hw_processes[hw_name] = process
                self.get_logger().info(f"Nodo {node_name} avviato (PID: {process.pid})")
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().error(f"Errore nell'avvio di {node_name}: {e}")
    
    def _cleanup_processes(self):
        """Clean up all launched processes"""
        for hw_name, process in self.hw_processes.items():
            try:
                if process.poll() is None: 
                    self.get_logger().info(f"Termino processo per {hw_name} (PID: {process.pid})")
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    try:
                        process.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                        process.wait(timeout=2.0)
            except Exception as e:
                self.get_logger().error(f"Errore nella terminazione del processo {hw_name}: {e}")

    def universal_callback(self, request, response, module_id):
        """Callback universale che smista le chiamate verso l'hardware del team"""
        self.get_logger().info(f"Richiesta UI per modulo [{module_id}]: {request.data}")

        client = self.hw_clients.get(module_id)

        if client is None or not client.service_is_ready():
            response.success = False
            response.message = f"Errore: Il servizio hardware per {module_id} non è disponibile."
            self.get_logger().error(response.message)
            return response

        hw_request = SetBool.Request()
        hw_request.data = request.data
        
        client.call_async(hw_request)

        response.success = True
        response.message = f"Comando inoltrato a {module_id}"
        
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
