from rclpy.node import Node


class MicrophoneNode(Node):
    def __init__(self):
        super().__init__('microphone_node')
        self.get_logger().info('Microphone Node Initialized')
