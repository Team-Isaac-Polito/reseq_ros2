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
        super().__init__("rplidar_mode")
        self.enable = False
        self.process = None
        
        # Crea il servizio ROS2
        self.srv = self.create_service(
            SetBool, 
            'rplidar_mode/toggle_scan', 
            self.handle_toggle
        )
        self.get_logger().info("Node RPLidar pronto. Servizio /rplidar_mode/toggle_scan attivo.")
    
    def handle_toggle(self, request, response):
        """Callback per il servizio ROS2."""
        self.enable = request.data
        
        if self.enable:
            response.success, response.message = self.start_rplidar()
        else:
            response.success, response.message = self.stop_rplidar()
        
        return response
    
    def start_rplidar(self):
        """Avvia il processo rplidar."""
        if self.process is not None and self.process.poll() is None:
            return True, "RPLidar già in esecuzione."
        
        try:
            self.get_logger().info(f"Avvio RPLidar: {' '.join(self.LAUNCH_CMD)}")
            self.process = subprocess.Popen(
                self.LAUNCH_CMD,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )
            
            # Aspetta un attimo per verificare che il processo sia partito
            time.sleep(0.5)
            if self.process.poll() is not None:
                stderr = self.process.stderr.read().decode('utf-8', errors='ignore')
                return False, f"RPLidar non è partito: {stderr[:200]}"
            
            self.get_logger().info("RPLidar avviato con successo.")
            return True, "RPLidar avviato."
        
        except Exception as e:
            self.get_logger().error(f"Errore nell'avvio di RPLidar: {e}")
            return False, f"Errore: {str(e)}"
    
    def stop_rplidar(self):
        """Ferma il processo rplidar."""
        if self.process is None or self.process.poll() is not None:
            return True, "RPLidar già fermato."
        
        try:
            self.get_logger().info("Arresto RPLidar...")
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            
            try:
                self.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warning("RPLidar non si è fermato, invio SIGKILL...")
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                self.process.wait(timeout=2.0)
            
            self.get_logger().info("RPLidar fermato.")
            self.process = None
            return True, "RPLidar fermato."
        
        except Exception as e:
            self.get_logger().error(f"Errore nell'arresto di RPLidar: {e}")
            return False, f"Errore: {str(e)}"
    
    def switchRplidar(self):
        """Legacy method - toggle manuale."""
        self.enable = not self.enable
        if self.enable:
            self.start_rplidar()
        else:
            self.stop_rplidar()


def main(args=None):
    """Entry point per il nodo."""
    rclpy.init(args=args)
    node = RplidarNode()
    try:
        rclpy.spin(node)
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