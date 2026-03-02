import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import cv2
import time

import board
import busio
import adafruit_mlx90640

class ThermalCameraNode(Node):
    def __init__(self):
        super().__init__('thermal_camera_node')

        # Setup I2C and sensor
        i2c = busio.I2C(board.SCL, board.SDA, frequency=800000)
        self.mlx = adafruit_mlx90640.MLX90640(i2c)
        self.mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ
        self.get_logger().info("MLX90640 detected with serial: {}".format(
            [hex(i) for i in self.mlx.serial_number]))

        # ROS publisher
        self.publisher_ = self.create_publisher(Image, '/thermal', 10)
        self.bridge = CvBridge()

        # Timer at 2Hz
        self.timer = self.create_timer(0.5, self.publish_thermal_image)

        # Buffer for sensor data
        self.frame = [0.0] * 768  # 32x24

    def publish_thermal_image(self):
        try:
            self.mlx.getFrame(self.frame)
        except ValueError:
            self.get_logger().warning("Failed to read frame, skipping.")
            return

        # Convert to NumPy array
        thermal_data = np.array(self.frame).reshape((24, 32)).astype(np.float32)

        # Normalize to 0-255 for visualization
        normalized = cv2.normalize(thermal_data, None, 0, 255, cv2.NORM_MINMAX)
        image_u8 = normalized.astype(np.uint8)

        image_u8 = cv2.flip(image_u8, 1)
        # Resize for better visibility (optional)
        # image_u8 = cv2.resize(image_u8, (320, 240), interpolation=cv2.INTER_NEAREST)

        # Convert to ROS image message
        msg = self.bridge.cv2_to_imgmsg(image_u8, encoding='mono8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info('Published thermal image.')

def main(args=None):
    rclpy.init(args=args)
    node = ThermalCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()