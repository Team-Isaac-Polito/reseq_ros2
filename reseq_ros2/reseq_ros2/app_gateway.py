import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from reseq_interfaces.srv import NodeStatus
import subprocess
import os
import signal
import time

class AppGateway(Node):
    def __init__(self):
        super().__init__('app_gateway')

        self.service_mapping = {
            'thermal': 'activate_thermal',
            'lidar': 'rplidar_mode/toggle_scan',
            'velocity': 'vel'
        }
        
        # Node launchers: mapping to how to launch each node
        self.node_launchers = {
            'thermal': ('thermal_node', ['ros2', 'run', 'reseq_ros2', 'thermal_node']),
            'lidar': ('rplidar_mode', ['ros2', 'run', 'reseq_ros2', 'rplidar_mode']),
            'velocity': ('velocity', ['ros2', 'run', 'reseq_ros2', 'velocity'])
        }
        
        self.hw_clients = {}
        self.hw_processes = {}  # Track running processes
        self.node_states = {
            'thermal': False,   # started or not
            'lidar': False,
            'velocity': False
        }
        
        # DO NOT auto-launch nodes - wait for app request
        # self._launch_dependent_nodes()  # REMOVED
        
        # Create UI services for each module
        for ui_name, hw_service in self.service_mapping.items():
            # service names on the module card of flutter app
            ui_service_name = f'ui/{ui_name}'
            self.create_service(SetBool, ui_service_name, 
                               lambda req, res, n=ui_name: self.universal_callback(req, res, n))
            self.hw_clients[ui_name] = self.create_client(SetBool, hw_service)
            self.get_logger().info(f"Mappato: {ui_service_name} -> {hw_service}")
        
        # Service to query node status
        self.create_service(NodeStatus, 'ui/node_status', self.handle_node_status)
        self.get_logger().info("Servizio /ui/node_status disponibile per query stato nodi")

        self.get_logger().info("Gateway App initialized correctly (lazy-loading mode).")
    
    def _ensure_node_running(self, module_id):
        """
        Ensure a node is running. Launch it if not already running.
        Returns: (success: bool, message: str)
        """
        if module_id not in self.node_launchers:
            return False, f"Modulo {module_id} non configurato"
        
        node_name, launch_cmd = self.node_launchers[module_id]
        
        # Check if process is still running
        if module_id in self.hw_processes and self.hw_processes[module_id] is not None:
            process = self.hw_processes[module_id]
            if process.poll() is None:  # Process still alive
                self.get_logger().info(f"Nodo {node_name} già in esecuzione (PID: {process.pid})")
                return True, f"{node_name} già attivo"
            else:
                # Process died, remove it
                del self.hw_processes[module_id]
                self.node_states[module_id] = False
        
        # Launch the node
        try:
            self.get_logger().info(f"Avvio on-demand nodo: {node_name}")
            process = subprocess.Popen(
                launch_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            self.hw_processes[module_id] = process
            self.node_states[module_id] = True
            self.get_logger().info(f"Nodo {node_name} avviato (PID: {process.pid})")
            
            # Give node time to initialize and register its service (2 seconds for safety)
            time.sleep(2.0)
            
            return True, f"{node_name} avviato con successo"
        except Exception as e:
            self.get_logger().error(f"Errore nell'avvio di {node_name}: {e}")
            self.node_states[module_id] = False
            return False, f"Errore nell'avvio: {str(e)}"
    
    def _stop_node(self, module_id):
        """
        Stop a running node.
        Returns: (success: bool, message: str)
        """
        if module_id not in self.node_launchers:
            return False, f"Modulo {module_id} non configurato"
        
        node_name, _ = self.node_launchers[module_id]
        
        if module_id not in self.hw_processes or self.hw_processes[module_id] is None:
            self.node_states[module_id] = False
            return True, f"{node_name} già fermo"
        
        process = self.hw_processes[module_id]
        if process.poll() is not None:  # Already dead
            self.node_states[module_id] = False
            del self.hw_processes[module_id]
            return True, f"{node_name} era già fermo"
        
        try:
            self.get_logger().info(f"Arresto nodo: {node_name} (PID: {process.pid})")
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            
            try:
                process.wait(timeout=3.0)
                self.get_logger().info(f"Nodo {node_name} fermato con SIGTERM")
            except subprocess.TimeoutExpired:
                self.get_logger().warning(f"Timeout SIGTERM per {node_name}, invio SIGKILL")
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait(timeout=2.0)
                self.get_logger().info(f"Nodo {node_name} fermato con SIGKILL")
            
            self.node_states[module_id] = False
            del self.hw_processes[module_id]
            return True, f"{node_name} fermato con successo"
        
        except Exception as e:
            self.get_logger().error(f"Errore nell'arresto di {node_name}: {e}")
            self.node_states[module_id] = False
            return False, f"Errore nell'arresto: {str(e)}"
    
    def _cleanup_processes(self):
        """Clean up all launched processes on shutdown"""
        for module_id in list(self.hw_processes.keys()):
            self._stop_node(module_id)

    def universal_callback(self, request, response, module_id):
        """
        Callback universale che gestisce le richieste UI.
        Se il nodo non è attivo, lo avvia.
        Poi invia la richiesta al nodo hardware.
        """
        self.get_logger().info(f"Richiesta UI per modulo [{module_id}]: {request.data}")
        
        # Se la richiesta è per accendere il modulo, assicurati che il nodo sia in esecuzione
        if request.data:
            success, msg = self._ensure_node_running(module_id)
            if not success:
                response.success = False
                response.message = f"Impossibile avviare il nodo: {msg}"
                self.get_logger().error(response.message)
                return response
            
            # Dai al nodo tempo per registrare il suo servizio
            time.sleep(0.5)
        
        # Verifica che il servizio hardware sia disponibile
        client = self.hw_clients.get(module_id)
        if client is None:
            response.success = False
            response.message = f"Client hardware per {module_id} non configurato"
            self.get_logger().error(response.message)
            return response
        
        # Aspetta che il servizio sia pronto
        if not client.service_is_ready():
            self.get_logger().warning(f"Servizio hardware {module_id} non pronto, riprovo...")
            for attempt in range(5):
                time.sleep(0.5)
                if client.service_is_ready():
                    break
            
            if not client.service_is_ready():
                response.success = False
                response.message = f"Errore: Il servizio hardware per {module_id} non è disponibile."
                self.get_logger().error(response.message)
                return response
        
        # Invia la richiesta al nodo hardware
        hw_request = SetBool.Request()
        hw_request.data = request.data
        
        try:
            future = client.call_async(hw_request)
            # Aspetta la risposta (con timeout di 10 secondi per permettere al nodo di startup)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                hw_response = future.result()
                response.success = hw_response.success
                response.message = hw_response.message
                self.get_logger().info(f"Risposta da {module_id}: {hw_response.message}")
            else:
                response.success = False
                response.message = f"Timeout nella comunicazione con {module_id}"
                self.get_logger().error(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Errore nella comunicazione: {str(e)}"
            self.get_logger().error(response.message)
        
        return response
    
    def handle_node_status(self, request, response):
        """
        Handler per il servizio /ui/node_status.
        Restituisce lo stato di tutti i nodi.
        """
        self.get_logger().info("Query stato nodi ricevuta")
        
        status_lines = []
        for module_id in ['thermal', 'lidar', 'velocity']:
            if module_id in self.hw_processes and self.hw_processes[module_id] is not None:
                process = self.hw_processes[module_id]
                is_running = process.poll() is None
                self.node_states[module_id] = is_running
            
            state = "RUNNING" if self.node_states.get(module_id, False) else "STOPPED"
            status_lines.append(f"{module_id}: {state}")
        
        response.status = "\n".join(status_lines)
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
