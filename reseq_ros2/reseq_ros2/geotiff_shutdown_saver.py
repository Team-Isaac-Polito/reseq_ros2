#!/usr/bin/env python3
"""
geotiff_shutdown_saver.py - Send savegeotiff command to geotiff_node on shutdown.

This node is launched via OnShutdown event handler to trigger a final
GeoTIFF save when the launch system is shutting down.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)
    node = Node('geotiff_shutdown_saver')
    
    # Create publisher for syscommand topic
    pub = node.create_publisher(String, 'syscommand', 10)
    
    # Wait briefly for publisher to be ready
    import time
    time.sleep(0.5)
    
    # Send savegeotiff command
    msg = String()
    msg.data = 'savegeotiff'
    pub.publish(msg)
    node.get_logger().info('Sent savegeotiff command to geotiff_node')
    
    # Give time for message to be delivered
    time.sleep(1.0)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()