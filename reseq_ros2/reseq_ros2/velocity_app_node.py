import rclpy
from rclpy.node import Node

"""
This node exposes the current velocity of motors in agevar.py to the app
I create a different node so that the normal node is not touch
get from vel_motors function
"""
class VelocityAppNode(Node):
    def __init__(self):
        super().__init__('velocity app node')
        self.velocity = 0
        self.get_logger().info("Velocirty app node started")
        

        