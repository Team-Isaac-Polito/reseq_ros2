import time

try:
    import adafruit_mlx90640
    import board
    import busio

    HAS_HARDWARE_LIBS = True
except ImportError:
    HAS_HARDWARE_LIBS = False

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool


class ThermalCameraNode(Node):
    def __init__(self):
        super().__init__('thermal_camera_node')

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter('frame_id', 'cameras_holder_link')
        self.declare_parameter('raw_topic', '/thermal/image_raw')
        self.declare_parameter('camera_info_topic', '/thermal/camera_info')
        self.declare_parameter('visualization_topic', '/thermal/image_visualization')
        self.declare_parameter('legacy_topic', '/thermal')
        self.declare_parameter('width', 32)
        self.declare_parameter('height', 24)
        self.declare_parameter('hfov_rad', 0.959931)  # 55 degrees horizontal FOV
        self.declare_parameter('flip_horizontal', True)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.raw_topic = self.get_parameter('raw_topic').get_parameter_value().string_value
        self.camera_info_topic = (
            self.get_parameter('camera_info_topic').get_parameter_value().string_value
        )
        self.vis_topic = (
            self.get_parameter('visualization_topic').get_parameter_value().string_value
        )
        self.legacy_topic = self.get_parameter('legacy_topic').get_parameter_value().string_value
        self.width = int(self.get_parameter('width').get_parameter_value().integer_value)
        self.height = int(self.get_parameter('height').get_parameter_value().integer_value)
        self.hfov_rad = float(self.get_parameter('hfov_rad').get_parameter_value().double_value)
        self.flip_horizontal = (
            self.get_parameter('flip_horizontal').get_parameter_value().bool_value
        )

        # ── Camera Intrinsics (Pinhole model for 32x24 sensor) ───────
        # fx = (width / 2) / tan(HFOV / 2)
        half_hfov = self.hfov_rad / 2.0
        self.fx = (self.width / 2.0) / np.tan(half_hfov)
        self.fy = self.fx  # Square pixels
        self.cx = (self.width - 1.0) / 2.0  # Center pixel X (15.5)
        self.cy = (self.height - 1.0) / 2.0  # Center pixel Y (11.5)

        self.is_active = False  # Start disabled
        self.mlx = None
        self.frame = [0.0] * (self.width * self.height)
        self.bridge = CvBridge()
        self.timer = None
        self.hardware_initialized = False

        # ── Publishers ───────────────────────────────────────────────
        self.pub_raw = self.create_publisher(Image, self.raw_topic, 10)
        self.pub_info = self.create_publisher(CameraInfo, self.camera_info_topic, 10)
        self.pub_vis = self.create_publisher(Image, self.vis_topic, 10)
        self.pub_legacy = self.create_publisher(Image, self.legacy_topic, 10)

        # Try to initialize hardware
        self._initialize_hardware()

        # ROS service for toggling camera on/off
        self.srv = self.create_service(SetBool, 'activate_thermal', self.handle_toggle_camera)
        self.get_logger().info(
            f'ThermalCameraNode ready. Publishing to [{self.raw_topic}], '
            f'[{self.camera_info_topic}], [{self.vis_topic}]. Service /activate_thermal active.'
        )

    def _initialize_hardware(self):
        """Initialize hardware with error handling"""
        if not HAS_HARDWARE_LIBS:
            self.get_logger().error(
                'adafruit_mlx90640 or board/busio not installed. Physical hardware unavailable.'
            )
            self.hardware_initialized = False
            return

        try:
            i2c = busio.I2C(board.SCL, board.SDA, frequency=800000)
            self.mlx = adafruit_mlx90640.MLX90640(i2c)
            self.mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_4_HZ
            self.get_logger().info(
                'MLX90640 detected with serial: {}'.format(
                    [hex(i) for i in self.mlx.serial_number]
                )
            )

            # Timer at 4Hz (only publishes when activated via service)
            self.timer = self.create_timer(0.25, self.publish_thermal_image)

            self.hardware_initialized = True
            self.get_logger().info('MLX90640 hardware initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Error initializing thermal hardware: {e}')
            self.hardware_initialized = False

    def handle_toggle_camera(self, request, response):
        """Handle camera on/off requests"""
        if not self.hardware_initialized:
            response.success = False
            response.message = 'Thermal hardware not available'
            self.get_logger().error(response.message)
            return response

        self.is_active = request.data

        if self.is_active:
            self.get_logger().info('Thermal camera activated')
            response.message = 'Camera activated successfully'
        else:
            self.get_logger().info('Thermal camera deactivated')
            response.message = 'Camera deactivated successfully'

        response.success = True
        return response

    def _build_camera_info(self, stamp) -> CameraInfo:
        """Construct sensor_msgs/CameraInfo matching MLX90640 resolution & FOV."""
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        # Intrinsic camera matrix K (3x3)
        info.k = [
            float(self.fx), 0.0, float(self.cx),
            0.0, float(self.fy), float(self.cy),
            0.0, 0.0, 1.0,
        ]
        # Rectification matrix R (3x3 identity)
        info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        # Projection matrix P (3x4)
        info.p = [
            float(self.fx), 0.0, float(self.cx), 0.0,
            0.0, float(self.fy), float(self.cy), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return info

    def publish_thermal_image(self):
        """Publish thermal data in 32FC1 (°C), CameraInfo, and colorized visualization."""
        if not self.is_active:
            return

        if not self.hardware_initialized or self.mlx is None:
            self.get_logger().warning('Hardware not initialized, cannot acquire frame')
            return

        try:
            self.mlx.getFrame(self.frame)
        except ValueError:
            self.get_logger().warning('Failed to read frame, skipping.')
            return
        except Exception as e:
            self.get_logger().error(f'Error reading sensor: {e}')
            return

        # 1. Convert frame to 2D array of temperatures in Celsius (float32)
        thermal_data = np.array(self.frame, dtype=np.float32).reshape((self.height, self.width))

        # 2. Match sensor mount orientation if flipped
        if self.flip_horizontal:
            thermal_data = cv2.flip(thermal_data, 1)

        stamp = self.get_clock().now().to_msg()

        # 3. Publish absolute temperature image (32FC1 in °C) for point cloud fusion & detection
        raw_msg = self.bridge.cv2_to_imgmsg(thermal_data, encoding='32FC1')
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = self.frame_id
        self.pub_raw.publish(raw_msg)

        # 4. Publish corresponding CameraInfo
        info_msg = self._build_camera_info(stamp)
        self.pub_info.publish(info_msg)

        # 5. Publish normalized & colormapped image (bgr8) for operator / RViz / Web UI
        norm_img = cv2.normalize(thermal_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        vis_bgr = cv2.applyColorMap(norm_img, cv2.COLORMAP_JET)
        vis_msg = self.bridge.cv2_to_imgmsg(vis_bgr, encoding='bgr8')
        vis_msg.header.stamp = stamp
        vis_msg.header.frame_id = self.frame_id
        self.pub_vis.publish(vis_msg)

        # 6. Legacy compatibility topic (/thermal mono8)
        legacy_msg = self.bridge.cv2_to_imgmsg(norm_img, encoding='mono8')
        legacy_msg.header.stamp = stamp
        legacy_msg.header.frame_id = self.frame_id
        self.pub_legacy.publish(legacy_msg)


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
