#!/usr/bin/env python3
"""Thermal–Pointcloud fusion node.

Fuses a depth/RGB-D PointCloud2 with a thermal Image to produce:
 - a temperature-augmented PointCloud2 in the global 'map' frame (/thermal/points)
 - localized, deduplicated hotspot markers in 'map' (/thermal/markers)
 - competition heat signature detections (/object_detection/detections)
 - optional birds-eye heatmap image (/fused/heatmap)

All geometric projections, coordinate transforms, and spatial clustering
are vectorised with NumPy for high throughput on Jetson.
"""

import math
import traceback
from typing import Dict, List

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray

try:
    from reseq_interfaces.msg import Detection

    HAS_DETECTION_MSG = True
except ImportError:
    HAS_DETECTION_MSG = False

try:
    from sklearn.cluster import DBSCAN

    HAS_SKLEARN = True
except ImportError:
    HAS_SKLEARN = False


class TrackedHotspot:
    def __init__(self, hotspot_id: int, pos: np.ndarray, peak_temp: float, stamp):
        self.id = hotspot_id
        self.pos = pos.copy()  # [x, y, z] in map frame
        self.peak_temp = float(peak_temp)
        self.last_seen = stamp
        self.hit_count = 1
        self.detection_sent = False


class ThermalPointcloudFusion(Node):
    """ROS 2 node that fuses depth point-clouds with thermal imagery."""

    def __init__(self):
        super().__init__('thermal_pointcloud_fusion')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('pointcloud_topic', '/realsense/depth/color/points')
        self.declare_parameter('thermal_image_topic', '/thermal/image_raw')
        self.declare_parameter('thermal_camera_info', '/thermal/camera_info')
        self.declare_parameter('camera_frame', 'cameras_holder_link')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('publish_fused_cloud_topic', '/thermal/points')
        self.declare_parameter('publish_local_cloud_topic', '/fused/points')
        self.declare_parameter('publish_heatmap_topic', '/fused/heatmap')
        self.declare_parameter('publish_markers_topic', '/thermal/markers')
        self.declare_parameter('detections_topic', '/object_detection/detections')

        self.declare_parameter('grid_resolution', 0.02)
        self.declare_parameter('grid_width', 400)
        self.declare_parameter('grid_height', 400)
        self.declare_parameter('grid_origin_x', -4.0)
        self.declare_parameter('grid_origin_y', -4.0)

        self.declare_parameter('use_camera_info', True)
        self.declare_parameter('intrinsics.fx', 30.7357)
        self.declare_parameter('intrinsics.fy', 30.7357)
        self.declare_parameter('intrinsics.cx', 15.5)
        self.declare_parameter('intrinsics.cy', 11.5)

        self.declare_parameter('temp_hotspot_threshold', 30.0)  # °C threshold for heat signature
        self.declare_parameter('cluster_radius', 0.15)  # 15 cm spatial clustering radius
        self.declare_parameter('min_cluster_points', 20)
        self.declare_parameter('dedup_distance', 0.25)  # 25 cm deduplication threshold
        self.declare_parameter('min_persistence_hits', 3)  # Consecutive frames to confirm hotspot
        self.declare_parameter('stale_timeout_sec', 1.0)  # Remove hotspots not observed for 1s
        self.declare_parameter('flip_horizontal', False)  
        self.declare_parameter('publish_heatmap', True)
        self.declare_parameter('tf_timeout_sec', 0.1)

        self.pc_topic = self._str_param('pointcloud_topic')
        self.thermal_topic = self._str_param('thermal_image_topic')
        self.camera_info_topic = self._str_param('thermal_camera_info')
        self.camera_frame = self._str_param('camera_frame')
        self.target_frame = self._str_param('target_frame')
        self.fused_cloud_topic = self._str_param('publish_fused_cloud_topic')
        self.local_cloud_topic = self._str_param('publish_local_cloud_topic')
        self.heatmap_topic = self._str_param('publish_heatmap_topic')
        self.markers_topic = self._str_param('publish_markers_topic')
        self.detections_topic = self._str_param('detections_topic')

        self.grid_resolution = self._double_param('grid_resolution')
        self.grid_w = int(self.get_parameter('grid_width').get_parameter_value().integer_value)
        self.grid_h = int(self.get_parameter('grid_height').get_parameter_value().integer_value)
        self.grid_origin_x = self._double_param('grid_origin_x')
        self.grid_origin_y = self._double_param('grid_origin_y')

        self.use_camera_info = (
            self.get_parameter('use_camera_info').get_parameter_value().bool_value
        )
        self.publish_heatmap_flag = (
            self.get_parameter('publish_heatmap').get_parameter_value().bool_value
        )
        self.tf_timeout_sec = self._double_param('tf_timeout_sec')

        self.temp_hotspot_threshold = self._double_param('temp_hotspot_threshold')
        self.cluster_radius = self._double_param('cluster_radius')
        self.min_cluster_points = int(
            self.get_parameter('min_cluster_points').get_parameter_value().integer_value
        )
        self.dedup_distance = self._double_param('dedup_distance')
        self.min_persistence_hits = int(
            self.get_parameter('min_persistence_hits').get_parameter_value().integer_value
        )
        self.stale_timeout_sec = self._double_param('stale_timeout_sec')
        self.flip_horizontal = (
            self.get_parameter('flip_horizontal').get_parameter_value().bool_value
        )

        self.fx = self._double_param('intrinsics.fx')
        self.fy = self._double_param('intrinsics.fy')
        self.cx = self._double_param('intrinsics.cx')
        self.cy = self._double_param('intrinsics.cy')

        # ── State ───────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.latest_thermal: np.ndarray | None = None
        self.latest_thermal_stamp = None
        self._last_processed_thermal_stamp = None
        self._last_process_time = self.get_clock().now()
        self.min_process_interval_sec = 0.15  # rate limit guard (sensor is ~4 Hz; 250ms period)
        self.tracked_hotspots: List[TrackedHotspot] = []
        self._next_hotspot_id = 1

        # ── TF ──────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscriptions ───────────────────────────────────────────
        qos = rclpy.qos.QoSProfile(depth=5)
        qos_pc = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, self.pc_topic, self._pc_cb, qos_pc)
        self.create_subscription(Image, self.thermal_topic, self._thermal_cb, qos)
        self._caminfo_sub = None
        if self.use_camera_info:
            self._caminfo_sub = self.create_subscription(
                CameraInfo, self.camera_info_topic, self._caminfo_cb, qos
            )

        # ── Publishers ──────────────────────────────────────────────
        self.pub_cloud = self.create_publisher(PointCloud2, self.fused_cloud_topic, qos)
        self.pub_local_cloud = self.create_publisher(PointCloud2, self.local_cloud_topic, qos)
        self.pub_vis = self.create_publisher(Image, '/thermal/image_visualization', qos)
        self.pub_heatmap = self.create_publisher(Image, self.heatmap_topic, qos)
        self.pub_markers = self.create_publisher(MarkerArray, self.markers_topic, qos)

        self.pub_detections = None
        if HAS_DETECTION_MSG:
            self.pub_detections = self.create_publisher(Detection, self.detections_topic, qos)

        self.get_logger().info(
            f'ThermalPointcloudFusion ready: PC [{self.pc_topic}] + '
            f'Thermal [{self.thermal_topic}] → [{self.fused_cloud_topic}] in frame [{self.target_frame}]'
        )

    # ── helpers ──────────────────────────────────────────────────────
    def _str_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_param(self, name: str) -> float:
        return float(self.get_parameter(name).get_parameter_value().double_value)

    # ── callbacks ────────────────────────────────────────────────────
    def _caminfo_cb(self, msg: CameraInfo):
        try:
            if msg.k[0] > 0.0:
                self.fx = float(msg.k[0])
                self.fy = float(msg.k[4])
                self.cx = float(msg.k[2])
                self.cy = float(msg.k[5])
                self.get_logger().info(
                    f'Intrinsics updated from CameraInfo: fx={self.fx:.2f} fy={self.fy:.2f} '
                    f'cx={self.cx:.2f} cy={self.cy:.2f}'
                )
                if self._caminfo_sub:
                    self.destroy_subscription(self._caminfo_sub)
                    self._caminfo_sub = None
        except Exception as e:
            self.get_logger().warn(f'Failed to parse CameraInfo: {e}')

    def _thermal_cb(self, msg: Image):
        """Receive thermal image and normalize to Celsius float32."""
        try:
            if msg.encoding in ('32FC1', '32FC'):
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            elif msg.encoding in ('16UC1', 'mono16'):
                raw16 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
                # Gazebo 16UC1 thermal camera typically encodes Kelvin or centi-Kelvin
                if raw16.max() > 1000:
                    cv_img = (raw16.astype(np.float32) / 100.0) - 273.15
                elif raw16.max() > 150:
                    cv_img = raw16.astype(np.float32) - 273.15
                else:
                    cv_img = raw16.astype(np.float32)
            elif msg.encoding in ('mono8', '8UC1'):
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8').astype(np.float32)
            else:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32)

            # If values appear to be in Kelvin (> 150), convert to Celsius
            if cv_img.size > 0 and cv_img.mean() > 150.0:
                cv_img = cv_img - 273.15

            if self.flip_horizontal and cv_img.size > 0:
                cv_img = cv2.flip(cv_img, 1)

            self.latest_thermal = cv_img
            self.latest_thermal_stamp = msg.header.stamp

            # Publish colormapped visualization image for RViz / operator
            if cv_img.size > 0:
                norm_th = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                vis_bgr = cv2.applyColorMap(norm_th, cv2.COLORMAP_JET)
                vis_msg = self.bridge.cv2_to_imgmsg(vis_bgr, encoding='bgr8')
                vis_msg.header.stamp = msg.header.stamp
                vis_msg.header.frame_id = self.camera_frame
                self.pub_vis.publish(vis_msg)
        except Exception as e:
            self.get_logger().error(f'Thermal convert error: {e}')

    # ── main point-cloud callback (vectorised) ───────────────────────
    def _pc_cb(self, pc_msg: PointCloud2):
        if self.latest_thermal is None or self.latest_thermal_stamp is None:
            return

        # Synchronize with thermal sensor: process at most once per fresh thermal frame.
        # This eliminates motion smear / ghost duplicates caused by fusing 30 Hz depth
        # clouds with stale 2 Hz thermal frames while the robot is moving.
        if self._last_processed_thermal_stamp is not None:
            if (
                self.latest_thermal_stamp.sec == self._last_processed_thermal_stamp.sec
                and self.latest_thermal_stamp.nanosec == self._last_processed_thermal_stamp.nanosec
            ):
                return

        now_time = self.get_clock().now()
        dt = (now_time - self._last_process_time).nanoseconds / 1e9
        if dt < self.min_process_interval_sec:
            return
        self._last_process_time = now_time
        self._last_processed_thermal_stamp = self.latest_thermal_stamp

        stamp = pc_msg.header.stamp
        timeout = Duration(seconds=self.tf_timeout_sec)

        # 1. Look up transform pc_frame → camera_frame (timestamped)
        try:
            tf_to_cam = self.tf_buffer.lookup_transform(
                self.camera_frame, pc_msg.header.frame_id, stamp, timeout=timeout
            )
        except TransformException:
            # Fallback to latest available transform if exact stamp is not yet in cache
            try:
                tf_to_cam = self.tf_buffer.lookup_transform(
                    self.camera_frame, pc_msg.header.frame_id, rclpy.time.Time(), timeout=timeout
                )
            except TransformException as e:
                self.get_logger().debug(f'TF to camera_frame failed: {e}')
                return

        # 2. Look up transform camera_frame → target_frame ('map')
        try:
            tf_to_map = self.tf_buffer.lookup_transform(
                self.target_frame, self.camera_frame, stamp, timeout=timeout
            )
        except TransformException:
            try:
                tf_to_map = self.tf_buffer.lookup_transform(
                    self.target_frame, self.camera_frame, rclpy.time.Time(), timeout=timeout
                )
            except TransformException as e:
                self.get_logger().debug(f'TF to map frame failed: {e}')
                return

        # 3. Read points → N×3 numpy array
        try:
            pts = pc2.read_points_numpy(pc_msg, field_names=['x', 'y', 'z'], skip_nans=True)
        except Exception:
            raw = np.array(list(pc2.read_points(pc_msg, field_names=('x', 'y', 'z'), skip_nans=True)))
            if raw.dtype.names:
                pts = np.column_stack([raw[k] for k in ('x', 'y', 'z')]).astype(np.float32)
            else:
                pts = raw.astype(np.float32)
        if pts.size == 0:
            return

        # Subsample input points if dense to keep CPU < 5%
        if len(pts) > 60000:
            pts = pts[::2]

        # 4. Transform points into camera_frame
        t_cam = tf_to_cam.transform.translation
        q_cam = tf_to_cam.transform.rotation
        T_cam = self._quat_to_matrix(q_cam.x, q_cam.y, q_cam.z, q_cam.w, t_cam.x, t_cam.y, t_cam.z)

        ones = np.ones((pts.shape[0], 1), dtype=np.float32)
        pts_h = np.hstack([pts, ones])
        pts_cam = (T_cam @ pts_h.T).T[:, :3]

        # 5. Perspective projection from robot camera frame (cameras_holder_link: X=forward, Y=left, Z=up)
        # Optical conventions: Z_opt = X (depth), X_opt = -Y (right), Y_opt = -Z (down)
        depth = pts_cam[:, 0]
        valid_depth = (depth > 0.1) & np.isfinite(depth)
        if not np.any(valid_depth):
            return

        pts_valid = pts_cam[valid_depth]
        d = depth[valid_depth]

        u = (self.fx * (-pts_valid[:, 1] / d) + self.cx).astype(np.int32)
        v = (self.fy * (-pts_valid[:, 2] / d) + self.cy).astype(np.int32)

        th_h, th_w = self.latest_thermal.shape[:2]
        in_bounds = (u >= 0) & (u < th_w) & (v >= 0) & (v < th_h)
        if not np.any(in_bounds):
            return

        u = u[in_bounds]
        v = v[in_bounds]
        xyz_cam = pts_valid[in_bounds]
        temps = self.latest_thermal[v, u]  # M temperatures in Celsius

        # 6. Transform projected points into target_frame ('map')
        t_map = tf_to_map.transform.translation
        q_map = tf_to_map.transform.rotation
        T_map = self._quat_to_matrix(q_map.x, q_map.y, q_map.z, q_map.w, t_map.x, t_map.y, t_map.z)

        ones_m = np.ones((xyz_cam.shape[0], 1), dtype=np.float32)
        xyz_cam_h = np.hstack([xyz_cam, ones_m])
        xyz_map = (T_map @ xyz_cam_h.T).T[:, :3]

        # 7. Publish temperature-augmented PointCloud2 in 'map' (zero-copy numpy)
        # Include both 'temperature' and 'intensity' so RViz automatically colors it
        fused_map = np.ascontiguousarray(np.column_stack([xyz_map, temps, temps]), dtype=np.float32)
        header_map = Header()
        header_map.stamp = stamp
        header_map.frame_id = self.target_frame

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='temperature', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_map_msg = pc2.create_cloud(header_map, fields, fused_map)
        self.pub_cloud.publish(cloud_map_msg)

        # 8. Optional: publish in camera frame as well (zero-copy numpy)
        fused_cam = np.ascontiguousarray(np.column_stack([xyz_cam, temps, temps]), dtype=np.float32)
        header_cam = Header()
        header_cam.stamp = stamp
        header_cam.frame_id = self.camera_frame
        local_cloud_msg = pc2.create_cloud(header_cam, fields, fused_cam)
        self.pub_local_cloud.publish(local_cloud_msg)

        # 9. Hotspot Clustering, Tracking, and Detection Integration
        self._process_hotspots(xyz_map, xyz_cam, temps, u, v, stamp)

        # 10. Optional birds-eye heatmap
        if self.publish_heatmap_flag:
            self._publish_heatmap(fused_map, header_map)

    # ── Hotspot detection & spatial clustering ─────────────────────────
    def _process_hotspots(
        self,
        xyz_map: np.ndarray,
        xyz_cam: np.ndarray,
        temps: np.ndarray,
        u_arr: np.ndarray,
        v_arr: np.ndarray,
        stamp,
    ):
        """Cluster points exceeding temp_hotspot_threshold, deduplicate, and publish markers/detections."""
        hot_mask = temps >= self.temp_hotspot_threshold
        if not np.any(hot_mask):
            self._publish_hotspot_markers(stamp)
            return

        hot_pts_map = xyz_map[hot_mask]
        hot_temps = temps[hot_mask]
        hot_pts_cam = xyz_cam[hot_mask]
        hot_u = u_arr[hot_mask]
        hot_v = v_arr[hot_mask]

        # Subsample hot points if dense to cap clustering execution to < 2ms
        n_hot = len(hot_pts_map)
        if n_hot < self.min_cluster_points:
            self._publish_hotspot_markers(stamp)
            return

        if n_hot > 400:
            step = int(np.ceil(n_hot / 400.0))
            hot_pts_map = hot_pts_map[::step]
            hot_temps = hot_temps[::step]
            hot_pts_cam = hot_pts_cam[::step]
            hot_u = hot_u[::step]
            hot_v = hot_v[::step]

        # DBSCAN clustering in 3D map frame
        if HAS_SKLEARN:
            db = DBSCAN(eps=self.cluster_radius, min_samples=self.min_cluster_points).fit(hot_pts_map)
            labels = db.labels_
        else:
            labels = self._simple_dbscan(hot_pts_map, self.cluster_radius, self.min_cluster_points)

        unique_labels = [lbl for lbl in np.unique(labels) if lbl >= 0]
        if not unique_labels:
            self._publish_hotspot_markers(stamp)
            return

        for lbl in unique_labels:
            c_mask = (labels == lbl)
            c_pts = hot_pts_map[c_mask]
            c_temps = hot_temps[c_mask]
            c_cam = hot_pts_cam[c_mask]
            c_u = hot_u[c_mask]
            c_v = hot_v[c_mask]

            centroid = np.mean(c_pts, axis=0)
            peak_temp = float(np.max(c_temps))
            avg_cam_depth = float(np.mean(c_cam[:, 0]))  # +X is forward depth in camera_frame

            # Check deduplication against tracked hotspots
            matched_th = None
            for th in self.tracked_hotspots:
                dist = np.linalg.norm(th.pos - centroid)
                if dist < self.dedup_distance:
                    # Update existing hotspot (running average on position)
                    th.pos = 0.3* th.pos + 0.7 * centroid
                    th.peak_temp = max(th.peak_temp, peak_temp)
                    th.last_seen = stamp
                    th.hit_count += 1
                    matched_th = th
                    break

            if matched_th is None:
                # Register new tracked hotspot candidate
                matched_th = TrackedHotspot(self._next_hotspot_id, centroid, peak_temp, stamp)
                self._next_hotspot_id += 1
                self.tracked_hotspots.append(matched_th)

            # Confirm and publish detection once persistence threshold is met
            if matched_th.hit_count >= self.min_persistence_hits and not matched_th.detection_sent:
                self.get_logger().info(
                    f'🔥 Confirmed Heat Signature #{matched_th.id} at '
                    f'[{matched_th.pos[0]:.2f}, {matched_th.pos[1]:.2f}, {matched_th.pos[2]:.2f}] '
                    f'with peak temp {matched_th.peak_temp:.1f}°C (hits={matched_th.hit_count})'
                )

                # Export detection to competition detection pipeline
                if self.pub_detections is not None:
                    u_min, u_max = int(np.min(c_u)), int(np.max(c_u))
                    v_min, v_max = int(np.min(c_v)), int(np.max(c_v))

                    det_msg = Detection()
                    det_msg.header.stamp = stamp
                    det_msg.header.frame_id = self.camera_frame
                    det_msg.detection = matched_th.id
                    det_msg.type = 'heat_sig'
                    det_msg.name = f'HeatSource_{matched_th.id}_{matched_th.peak_temp:.1f}C'
                    det_msg.robot = 'reseq'
                    det_msg.mode = 'mapping'
                    det_msg.confidence = float(min(1.0, max(0.5, (matched_th.peak_temp - self.temp_hotspot_threshold) / 20.0 + 0.5)))
                    det_msg.xmin = u_min
                    det_msg.ymin = v_min
                    det_msg.width = max(1, u_max - u_min)
                    det_msg.height = max(1, v_max - v_min)
                    det_msg.depth_center = avg_cam_depth
                    det_msg.camera_frame = self.camera_frame

                    self.pub_detections.publish(det_msg)
                    matched_th.detection_sent = True

        self._publish_hotspot_markers(stamp)

    def _publish_hotspot_markers(self, stamp):
        """Publish deduplicated hotspot spheres and text labels to RViz; prune stale markers."""
        marker_array = MarkerArray()
        now_time = self.get_clock().now()

        active_hotspots = []
        for th in self.tracked_hotspots:
            age = (now_time - rclpy.time.Time.from_msg(th.last_seen)).nanoseconds / 1e9
            if age > self.stale_timeout_sec:
                # Remove stale marker from RViz if it was displayed
                if th.hit_count >= self.min_persistence_hits:
                    del_sphere = Marker()
                    del_sphere.header.stamp = stamp
                    del_sphere.header.frame_id = self.target_frame
                    del_sphere.ns = 'thermal_hotspots'
                    del_sphere.id = th.id * 2
                    del_sphere.action = Marker.DELETE
                    marker_array.markers.append(del_sphere)

                    del_text = Marker()
                    del_text.header.stamp = stamp
                    del_text.header.frame_id = self.target_frame
                    del_text.ns = 'thermal_hotspot_labels'
                    del_text.id = th.id * 2 + 1
                    del_text.action = Marker.DELETE
                    marker_array.markers.append(del_text)
            else:
                active_hotspots.append(th)

                # Only render markers for confirmed hotspots
                if th.hit_count >= self.min_persistence_hits:
                    # Sphere marker
                    sphere = Marker()
                    sphere.header.stamp = stamp
                    sphere.header.frame_id = self.target_frame
                    sphere.ns = 'thermal_hotspots'
                    sphere.id = th.id * 2
                    sphere.type = Marker.SPHERE
                    sphere.action = Marker.ADD
                    sphere.pose.position.x = float(th.pos[0])
                    sphere.pose.position.y = float(th.pos[1])
                    sphere.pose.position.z = float(th.pos[2])
                    sphere.pose.orientation.w = 1.0
                    sphere.scale.x = 0.25
                    sphere.scale.y = 0.25
                    sphere.scale.z = 0.25

                    # Temperature-based color: orange to red
                    temp_ratio = min(1.0, max(0.0, (th.peak_temp - self.temp_hotspot_threshold) / 30.0))
                    sphere.color = ColorRGBA(r=1.0, g=float(1.0 - temp_ratio * 0.7), b=0.0, a=0.85)
                    marker_array.markers.append(sphere)

                    # Text label marker
                    text = Marker()
                    text.header.stamp = stamp
                    text.header.frame_id = self.target_frame
                    text.ns = 'thermal_hotspot_labels'
                    text.id = th.id * 2 + 1
                    text.type = Marker.TEXT_VIEW_FACING
                    text.action = Marker.ADD
                    text.pose.position.x = float(th.pos[0])
                    text.pose.position.y = float(th.pos[1])
                    text.pose.position.z = float(th.pos[2] + 0.3)
                    text.pose.orientation.w = 1.0
                    text.scale.z = 0.14
                    text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                    text.text = f'Heat #{th.id}: {th.peak_temp:.1f}°C'
                    marker_array.markers.append(text)

        self.tracked_hotspots = active_hotspots
        if marker_array.markers:
            self.pub_markers.publish(marker_array)

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

        heatmap_colour = cv2.applyColorMap(scaled, cv2.COLORMAP_JET)
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

    @staticmethod
    def _simple_dbscan(pts: np.ndarray, eps: float, min_samples: int) -> np.ndarray:
        """Pure NumPy/Python BFS DBSCAN fallback with bounded deque."""
        from collections import deque
        n = len(pts)
        labels = np.full(n, -1, dtype=int)
        cluster_id = 0
        visited = np.zeros(n, dtype=bool)
        in_queue = np.zeros(n, dtype=bool)
        eps_sq = eps * eps

        for i in range(n):
            if visited[i]:
                continue
            visited[i] = True

            diffs = pts - pts[i]
            dists_sq = np.sum(diffs * diffs, axis=1)
            neighbors = np.where(dists_sq <= eps_sq)[0]

            if len(neighbors) < min_samples:
                labels[i] = -1
            else:
                labels[i] = cluster_id
                q = deque()
                for nb in neighbors:
                    if nb != i and not visited[nb] and not in_queue[nb]:
                        in_queue[nb] = True
                        q.append(nb)

                while q:
                    curr = q.popleft()
                    visited[curr] = True
                    labels[curr] = cluster_id

                    c_diffs = pts - pts[curr]
                    c_dists_sq = np.sum(c_diffs * c_diffs, axis=1)
                    c_neighbors = np.where(c_dists_sq <= eps_sq)[0]
                    if len(c_neighbors) >= min_samples:
                        for c_nb in c_neighbors:
                            if not visited[c_nb] and not in_queue[c_nb]:
                                in_queue[c_nb] = True
                                q.append(c_nb)
                cluster_id += 1
        return labels


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
