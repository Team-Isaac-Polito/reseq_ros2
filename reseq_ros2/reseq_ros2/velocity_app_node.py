import rclpy
from rclpy.node import Node

"""
This node exposes the current motors' velocity from agevar.py to the mobile app.
It is implemented as a separate node so the main control node is not modified.
It obtains values from the `vel_motors` function.
"""


class VelocityAppNode(Node):
    def __init__(self):
        super().__init__('velocity_app_node')
        self.velocity = 0
        self.get_logger().info('Velocity app node started')
