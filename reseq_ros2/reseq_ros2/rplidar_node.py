import os
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, SetBool


class RplidarNode(Node):
    """Node that manages rplidar start/stop via SetBool service."""
    
    LAUNCH_CMD = ['ros2', 'launch', 'rplidar_ros', 'rplidar_a2m8_launch.py']

    def __init__(self):
        super().__init__('rplidar_mode')
        self.process = None
        self.enable = False

        self.srv = self.create_service(
            SetBool,
            'rplidar_mode/toggle_scan',
            self.handle_toggle
        )

        self.start_motor_cli = self.create_client(Empty, 'start_motor')
        self.stop_motor_cli = self.create_client(Empty, 'stop_motor')
        self.get_logger().info('Wrapper Lpidar inizializzato')
    
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
            return True, 'Rplidar is running'

        try:
            self.get_logger().info("Avvio del driver RPLidar via subprocess...")
            self.process = subprocess.Popen(
                self.LAUNCH_CMD,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )

            self.get_logger().info("Attesa attivazione servizi hardware...")
            time.sleep(3.0) 

            if self.start_motor_cli.wait_for_service(timeout_sec=5.0):
                self.start_motor_cli.call_async(Empty.Request())
                return True, "Driver avviato e rotazione motore attivata."
            else:
                return True, "Driver avviato, ma i servizi motore non rispondono (timeout)."

        except Exception as e:
            self.get_logger().error(f"Errore critico avvio: {str(e)}")
            return False, f"Errore nell'avvio del processo: {str(e)}"

    
    def stop_rplidar(self):
        """Ferma il motore e chiude il driver."""
        if self.process is None:
            return True, 'RPLidar non era attivo.'

        try:
            if self.stop_motor_cli.wait_for_service(timeout_sec=1.0):
                self.stop_motor_cli.call_async(Empty.Request())
                time.sleep(0.5)

            self.get_logger().info('Arresto del driver hardware...')
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=2.0)
            self.process = None
            return True, "RPLidar spento correttamente."
            
        except Exception as e:
            return False, f"Errore durante lo spegnimento: {str(e)}"


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