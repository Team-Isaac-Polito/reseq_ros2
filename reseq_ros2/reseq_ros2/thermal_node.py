import time

import adafruit_mlx90640
import board
import busio
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool


class ThermalCameraNode(Node):
    def __init__(self):
        super().__init__('thermal_camera_node')

        self.is_active = False  # Start disabled
        self.mlx = None
        self.frame = [0.0] * 768  # 32x24
        self.publisher_ = None
        self.bridge = CvBridge()
        self.timer = None
        self.hardware_initialized = False

        # Try to initialize hardware
        self._initialize_hardware()

        # ROS service for toggling camera on/off
        self.srv = self.create_service(SetBool, 'activate_thermal', self.handle_toggle_camera)
        self.get_logger().info('Nodo Termico pronto. Servizio /activate_thermal attivo.')

    def _initialize_hardware(self):
        """Initialize hardware with error handling"""
        try:
            i2c = busio.I2C(board.SCL, board.SDA, frequency=800000)
            self.mlx = adafruit_mlx90640.MLX90640(i2c)
            self.mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ
            self.get_logger().info(
                'MLX90640 detected with serial: {}'.format(
                    [hex(i) for i in self.mlx.serial_number]
                )
            )

            # ROS publisher - only create if hardware is ready
            self.publisher_ = self.create_publisher(Image, '/thermal', 10)
            self.bridge = CvBridge()

            # Timer at 2Hz (but only publishes if active)
            self.timer = self.create_timer(0.5, self.publish_thermal_image)

            self.hardware_initialized = True
            self.get_logger().info('Hardware inizializzato correttamente')
        except Exception as e:
            self.get_logger().error(f"Errore nell'inizializzazione dell'hardware termico: {e}")
            self.hardware_initialized = False

    def handle_toggle_camera(self, request, response):
        """Handle camera on/off requests"""
        if not self.hardware_initialized:
            response.success = False
            response.message = 'Hardware termico non disponibile'
            self.get_logger().error(response.message)
            return response

        self.is_active = request.data

        if self.is_active:
            self.get_logger().info('Videocamera termica attivata')
            response.message = 'Camera attivata con successo'
        else:
            self.get_logger().info('Videocamera termica disattivata')
            response.message = 'Camera disattivata con successo'

        response.success = True
        return response

    def publish_thermal_image(self):
        """Publish thermal image if camera is active"""
        if not self.is_active:
            return

        if not self.hardware_initialized or self.mlx is None:
            self.get_logger().warning('Hardware non inizializzato, impossibile acquisire frame')
            return

        try:
            self.mlx.getFrame(self.frame)
        except ValueError:
            self.get_logger().warning('Failed to read frame, skipping.')
            return
        except Exception as e:
            self.get_logger().error(f'Errore nella lettura del sensore: {e}')
            return

        # Convert to NumPy array
        thermal_data = np.array(self.frame).reshape((24, 32)).astype(np.float32)

        # Normalize to 0-255 for visualization
        normalized = cv2.normalize(thermal_data, None, 0, 255, cv2.NORM_MINMAX)
        image_u8 = normalized.astype(np.uint8)

        image_u8 = cv2.flip(image_u8, 1)

        # Convert to ROS image message
        msg = self.bridge.cv2_to_imgmsg(image_u8, encoding='mono8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info('Published thermal image.')


def main(args=None):
    rclpy.init(args=args)
    node = ThermalCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
