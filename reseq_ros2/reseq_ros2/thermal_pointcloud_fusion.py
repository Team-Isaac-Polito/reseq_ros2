#!/usr/bin/env python3
"""Thermal–Pointcloud fusion node.

Fuses a stereo/depth PointCloud2 with a thermal Image to produce:
 - a fused PointCloud2 with an extra ``temperature`` field (float32)
 - a 2-D heatmap image (birds-eye grid of temperatures)

The node uses TF2 to transform the point cloud into the thermal camera's
frame, projects each point through a pinhole model to look up the
corresponding pixel temperature, then publishes the augmented cloud and an
optional birds-eye heatmap.

All heavy-lifting (projection, masking, grid accumulation) is vectorised
with NumPy so performance is acceptable even on embedded Jetson platforms.
"""

import math
import traceback

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros import TransformException


class ThermalPointcloudFusion(Node):
    """ROS 2 node that fuses depth point-clouds with thermal imagery."""

    def __init__(self):
        super().__init__('thermal_pointcloud_fusion')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('pointcloud_topic', '/camera/depth/color/points')
        self.declare_parameter('thermal_image_topic', '/thermal/image_raw')
        self.declare_parameter('thermal_camera_info', '/thermal/camera_info')
        self.declare_parameter('camera_frame', 'thermal_frame')
        self.declare_parameter('publish_fused_cloud_topic', '/fused/points')
        self.declare_parameter('publish_heatmap_topic', '/fused/heatmap')
        self.declare_parameter('grid_resolution', 0.01)
        self.declare_parameter('grid_width', 500)
        self.declare_parameter('grid_height', 500)
        self.declare_parameter('grid_origin_x', -2.5)
        self.declare_parameter('grid_origin_y', -2.5)
        self.declare_parameter('use_camera_info', True)
        self.declare_parameter('intrinsics.fx', 300.0)
        self.declare_parameter('intrinsics.fy', 300.0)
        self.declare_parameter('intrinsics.cx', 160.0)
        self.declare_parameter('intrinsics.cy', 120.0)
        self.declare_parameter('publish_heatmap', True)

        self.pc_topic = self._str_param('pointcloud_topic')
        self.thermal_topic = self._str_param('thermal_image_topic')
        self.camera_info_topic = self._str_param('thermal_camera_info')
        self.camera_frame = self._str_param('camera_frame')
        self.fused_cloud_topic = self._str_param('publish_fused_cloud_topic')
        self.heatmap_topic = self._str_param('publish_heatmap_topic')
        self.grid_resolution = self._double_param('grid_resolution')
        self.grid_w = int(self.get_parameter('grid_width').get_parameter_value().integer_value)
        self.grid_h = int(self.get_parameter('grid_height').get_parameter_value().integer_value)
        self.grid_origin_x = self._double_param('grid_origin_x')
        self.grid_origin_y = self._double_param('grid_origin_y')
        self.use_camera_info = self.get_parameter('use_camera_info').get_parameter_value().bool_value
        self.publish_heatmap_flag = self.get_parameter('publish_heatmap').get_parameter_value().bool_value

        self.fx = self._double_param('intrinsics.fx')
        self.fy = self._double_param('intrinsics.fy')
        self.cx = self._double_param('intrinsics.cx')
        self.cy = self._double_param('intrinsics.cy')

        # ── State ───────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.latest_thermal: np.ndarray | None = None

        # ── TF ──────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscriptions ───────────────────────────────────────────
        qos = rclpy.qos.QoSProfile(depth=5)
        self.create_subscription(PointCloud2, self.pc_topic, self._pc_cb, qos)
        self.create_subscription(Image, self.thermal_topic, self._thermal_cb, qos)
        self._caminfo_sub = None
        if self.use_camera_info:
            self._caminfo_sub = self.create_subscription(
                CameraInfo, self.camera_info_topic, self._caminfo_cb, qos
            )

        # ── Publishers ──────────────────────────────────────────────
        self.pub_cloud = self.create_publisher(PointCloud2, self.fused_cloud_topic, qos)
        self.pub_heatmap = self.create_publisher(Image, self.heatmap_topic, qos)

        self.get_logger().info(
            f'ThermalPointcloudFusion ready  '
            f'PC: {self.pc_topic} → {self.fused_cloud_topic}  '
            f'Thermal: {self.thermal_topic}'
        )

    # ── helpers ──────────────────────────────────────────────────────
    def _str_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_param(self, name: str) -> float:
        return float(self.get_parameter(name).get_parameter_value().double_value)

    # ── callbacks ────────────────────────────────────────────────────
    def _caminfo_cb(self, msg: CameraInfo):
        try:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(
                f'Intrinsics from CameraInfo: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f}'
            )
            if self._caminfo_sub:
                self.destroy_subscription(self._caminfo_sub)
                self._caminfo_sub = None
        except Exception as e:
            self.get_logger().warn(f'Failed to parse CameraInfo: {e}')

    def _thermal_cb(self, msg: Image):
        try:
            if msg.encoding in ('mono8', '8UC1'):
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            elif msg.encoding in ('16UC1', 'mono16'):
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
            else:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_thermal = np.asarray(cv_img, dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f'Thermal convert error: {e}')

    # ── main point-cloud callback (vectorised) ───────────────────────
    def _pc_cb(self, pc_msg: PointCloud2):
        if self.latest_thermal is None:
            return

        # 1. Look up transform  pc_frame → thermal_frame
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.camera_frame, pc_msg.header.frame_id, rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warn(f'TF failed: {e}', throttle_duration_sec=5.0)
            return

        # 2. Read points → N×3 numpy
        pts_gen = pc2.read_points(pc_msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.array(list(pts_gen), dtype=np.float32)
        if pts.size == 0:
            return

        # 3. Build 4×4 transform matrix from TF
        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        T = self._quat_to_matrix(q.x, q.y, q.z, q.w, t.x, t.y, t.z)

        # 4. Transform points into thermal frame  (N×3)
        ones = np.ones((pts.shape[0], 1), dtype=np.float32)
        pts_h = np.hstack([pts, ones])  # N×4
        pts_tf = (T @ pts_h.T).T[:, :3]  # N×3

        # 5. Perspective projection  u = fx*X/Z + cx,  v = fy*Y/Z + cy
        Z = pts_tf[:, 2]
        valid = (Z > 0) & np.isfinite(Z)
        X = pts_tf[valid, 0]
        Y = pts_tf[valid, 1]
        Z = Z[valid]

        u = (self.fx * (X / Z) + self.cx).astype(np.int32)
        v = (self.fy * (Y / Z) + self.cy).astype(np.int32)

        th_h, th_w = self.latest_thermal.shape[:2]
        in_bounds = (u >= 0) & (u < th_w) & (v >= 0) & (v < th_h)

        u = u[in_bounds]
        v = v[in_bounds]
        xyz = pts_tf[valid][in_bounds]  # M×3

        if xyz.shape[0] == 0:
            return

        temps = self.latest_thermal[v, u]  # M temperatures

        # 6. Build & publish fused cloud
        fused = np.column_stack([xyz, temps])  # M×4
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.camera_frame
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='temperature', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg = pc2.create_cloud(header, fields, fused.tolist())
        self.pub_cloud.publish(cloud_msg)

        # 7. Optional birds-eye heatmap
        if self.publish_heatmap_flag:
            self._publish_heatmap(fused, header)

        self.get_logger().debug(f'Fused cloud: {fused.shape[0]} pts')

    # ── heatmap grid (vectorised) ────────────────────────────────────
    def _publish_heatmap(self, fused: np.ndarray, header: Header):
        gx = np.floor((fused[:, 0] - self.grid_origin_x) / self.grid_resolution).astype(np.int32)
        gy = np.floor((fused[:, 1] - self.grid_origin_y) / self.grid_resolution).astype(np.int32)
        mask = (gx >= 0) & (gx < self.grid_w) & (gy >= 0) & (gy < self.grid_h)
        gx, gy, temps = gx[mask], gy[mask], fused[mask, 3]

        if gx.size == 0:
            return

        grid_sum = np.zeros((self.grid_h, self.grid_w), dtype=np.float64)
        grid_cnt = np.zeros((self.grid_h, self.grid_w), dtype=np.int32)
        np.add.at(grid_sum, (gy, gx), temps.astype(np.float64))
        np.add.at(grid_cnt, (gy, gx), 1)

        has_data = grid_cnt > 0
        grid_mean = np.zeros_like(grid_sum)
        grid_mean[has_data] = grid_sum[has_data] / grid_cnt[has_data]

        vmin, vmax = grid_mean[has_data].min(), grid_mean[has_data].max()
        if vmax - vmin < 1e-6:
            vmax = vmin + 1.0
        scaled = np.zeros((self.grid_h, self.grid_w), dtype=np.uint8)
        scaled[has_data] = np.clip(
            (grid_mean[has_data] - vmin) / (vmax - vmin) * 255.0, 0, 255
        ).astype(np.uint8)

        # Apply a colour-map for nicer visualisation
        heatmap_colour = cv2.applyColorMap(scaled, cv2.COLORMAP_JET)
        # Keep cells without data black
        heatmap_colour[~has_data] = 0

        msg = self.bridge.cv2_to_imgmsg(heatmap_colour, encoding='bgr8')
        msg.header = header
        self.pub_heatmap.publish(msg)

    # ── quaternion → 4×4 homogeneous matrix ─────────────────────────
    @staticmethod
    def _quat_to_matrix(
        qx: float, qy: float, qz: float, qw: float,
        tx: float, ty: float, tz: float,
    ) -> np.ndarray:
        """Convert quaternion + translation to a 4×4 transform matrix."""
        # rotation
        x2 = qx + qx
        y2 = qy + qy
        z2 = qz + qz
        xx = qx * x2
        xy = qx * y2
        xz = qx * z2
        yy = qy * y2
        yz = qy * z2
        zz = qz * z2
        wx = qw * x2
        wy = qw * y2
        wz = qw * z2

        T = np.eye(4, dtype=np.float32)
        T[0, 0] = 1.0 - (yy + zz)
        T[0, 1] = xy - wz
        T[0, 2] = xz + wy
        T[1, 0] = xy + wz
        T[1, 1] = 1.0 - (xx + zz)
        T[1, 2] = yz - wx
        T[2, 0] = xz - wy
        T[2, 1] = yz + wx
        T[2, 2] = 1.0 - (xx + yy)
        T[0, 3] = tx
        T[1, 3] = ty
        T[2, 3] = tz
        return T


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ThermalPointcloudFusion()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as err:
        if node:
            node.get_logger().fatal(
                f'Error in ThermalPointcloudFusion: {err}\n{traceback.format_exc()}'
            )
    finally:
        if node:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
