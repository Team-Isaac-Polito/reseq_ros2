from rclpy.node import Node
from std_srvs.srv import SetBool
import subprocess
import rclpy
from rclpy.executors import MultiThreadedExecutor


class MicrophoneNode(Node):
    def __init__(self):
        super().__init__('microphone_node')
        self.get_logger().info('Microphone Node Initialized')
        self.srv = self.create_service(SetBool, '/activate_microphone', self.toggle_audio_callback)
        self.audio_process = None

    def toggle_audio_callback(self, request, response):
        # We sent an activation request
        if request.data:
            if self.audio_process is None:
                self.get_logger().info('Staerting C++ audio process')
                self.audio_process = subprocess.Popen([
                    'ros2', 'run', 'audio_capture', 'audio_capture_node',
                    '--ros-args', '-p', 'format:=mp3'
                ])
                response.success = True
                response.message = 'Microphone activated'
            else:
                response.success = True
                response.message = 'Driver già alredy in execution.'
        else:
            if self.audio_process:
                self.get_logger().info('Terminating C++ audio process')
                self.audio_process.terminate()
                self.audio_process = None
                response.success = True
                response.message = 'Microphone deactivated'
        return response
    
    def destroy_audio_node(self):
        if self.audio_process:
            self.get_logger().info('Terminating C++ audio process')
            self.audio_process.terminate()
            try:
                self.audio_process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.audio_process.kill()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MicrophoneNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_audio_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
