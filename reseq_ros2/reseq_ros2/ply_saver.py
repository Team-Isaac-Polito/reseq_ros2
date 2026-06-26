"""
ply_saver.py - Accumulate colored PointCloud2 frames into a single PLY map.

Subscribes to a colored PointCloud2 topic (RGBD camera or RealSense),
transforms each frame into the map frame via TF2, accumulates points in a
voxel grid (latest colour per voxel), and periodically writes reseq_map_3d.ply.

Works on both real robot and Gazebo simulation with the same topic names.
The Gazebo bridge (gz_bridge.yaml) publishes to the same /camera/... topics
that the physical RealSense driver produces.

QoS: uses RELIABLE to match the ros_gz_bridge publisher. The RealSense driver
with pointcloud.enable:=true also uses RELIABLE, so this works on both platforms.
"""

from pathlib import Path
from std_msgs.msg import Header

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs.msg import PointField


def _parse_xyz_rgb(msg: PointCloud2):
    """Extract XYZ (float32) and RGB (uint8) from a colored PointCloud2.

    The RGB field may be stored as a single float32 (Gazebo bridge packs
    RGBA into one float) or as separate uint8 values (RealSense native).
    Returns (pts_xyz (N,3) float64, pts_rgb (N,3) uint8) or (None, None).
    """
    fields = {f.name: (f.offset, f.datatype) for f in msg.fields}
    if not all(k in fields for k in ('x', 'y', 'z', 'rgb')):
        return None, None

    ox = fields['x'][0]
    o_rgb, rgb_dt = fields['rgb']
    step = msg.point_step
    n = msg.width * msg.height

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)

    xyz = raw[:, ox : ox + 12].copy().view(np.float32).reshape(n, 3).astype(np.float64)

    if rgb_dt == 7:  # FLOAT32 - Gazebo bridge packs RGBA into one float32
        rgb = raw[:, o_rgb : o_rgb + 4].astype(np.uint8)[:, :3].copy()
    else:  # UINT8, UINT32, etc. - RealSense native format
        rgb = raw[:, o_rgb : o_rgb + 3].copy()

    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid], rgb[valid]


def _tf_to_Rt(tf_stamped):
    """Return (R 3x3 float64, t 3-vec float64) from a TF stamped transform."""
    tr = tf_stamped.transform.translation
    q = tf_stamped.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    t = np.array([tr.x, tr.y, tr.z], dtype=np.float64)
    return R, t


def _write_ply_rgb(path: Path, pts: np.ndarray, colors: np.ndarray) -> int:
    """Write ASCII PLY with XYZ+RGB. Returns vertex count."""
    path.parent.mkdir(parents=True, exist_ok=True)
    n = len(pts)
    with path.open('w') as f:
        f.write(f'ply\nformat ascii 1.0\nelement vertex {n}\n')
        f.write('property float x\nproperty float y\nproperty float z\n')
        f.write('property uchar red\nproperty uchar green\nproperty uchar blue\n')
        f.write('end_header\n')
        data = np.column_stack((pts, colors))
        np.savetxt(f, data, fmt='%.4f %.4f %.4f %d %d %d')
    return n


class PlySaver(Node):
    def __init__(self) -> None:
        super().__init__('ply_saver')

        self.declare_parameter('pointcloud_topic', '/camera/depth/color/points')
        self.declare_parameter('save_path', '/ros2_ws/maps')
        self.declare_parameter('save_interval', 60.0)
        self.declare_parameter('publish_interval', 5.0)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('frame_skip', 5)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_frame_skip', 2)
        self.declare_parameter('scan_max_range', 12.0)
        self.declare_parameter('scan_color_rgb', [255, 210, 0])

        self._pc_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        save_dir = Path(self.get_parameter('save_path').get_parameter_value().string_value)
        interval = self.get_parameter('save_interval').get_parameter_value().double_value
        publish_interval = (
            self.get_parameter('publish_interval').get_parameter_value().double_value
        )
        self._voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self._frame_skip = self.get_parameter('frame_skip').get_parameter_value().integer_value
        self._max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self._scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self._scan_frame_skip = (
            self.get_parameter('scan_frame_skip').get_parameter_value().integer_value
        )
        self._scan_max_range = (
            self.get_parameter('scan_max_range').get_parameter_value().double_value
        )
        scan_rgb = self.get_parameter('scan_color_rgb').value
        self._scan_rgb = tuple(int(max(0, min(255, c))) for c in scan_rgb[:3])

        self._ply_path = save_dir / 'reseq_map_3d.ply'
        self._map_topic = '/ply_map/points'
        self._voxel_map = {}
        self._frame_idx: int = 0
        self._scan_frame_idx: int = 0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        pc_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            PointCloud2,
            self._pc_topic,
            self._pc_cb,
            pc_qos,
        )
        self.create_subscription(
            LaserScan,
            self._scan_topic,
            self._scan_cb,
            scan_qos,
        )

        self._map_pub = self.create_publisher(PointCloud2, self._map_topic, pc_qos)

        self.create_timer(interval, self._save)
        self.create_timer(publish_interval, self._publish_map_cloud)
        self.get_logger().info(
            f'PlySaver listening on [{self._pc_topic}] and [{self._scan_topic}], '
            f'save every {interval:.0f}s -> {self._ply_path}'
        )

    def _accumulate_pts(self, pts_map: np.ndarray, rgb: np.ndarray) -> None:
        if len(pts_map) == 0:
            return
        vs = self._voxel_size
        voxel_keys = tuple(np.floor(pts_map[:, i] / vs).astype(np.int64) for i in range(3))
        for k0, k1, k2, c in zip(*voxel_keys, rgb):
            self._voxel_map[(int(k0), int(k1), int(k2))] = (int(c[0]), int(c[1]), int(c[2]))

    def _voxel_map_to_pointcloud(self) -> PointCloud2 | None:
        if not self._voxel_map:
            return None

        vs = self._voxel_size
        items = list(self._voxel_map.items())
        keys = np.array([k for k, _ in items], dtype=np.int64)
        colors = np.array([v for _, v in items], dtype=np.uint8)
        pts = keys.astype(np.float64) * vs + vs / 2.0

        rgb_uint32 = (
            (colors[:, 0].astype(np.uint32) << 16)
            | (colors[:, 1].astype(np.uint32) << 8)
            | colors[:, 2].astype(np.uint32)
        )
        rgb_float = rgb_uint32.view(np.float32)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_rows = np.column_stack((pts.astype(np.float32), rgb_float.astype(np.float32)))
        return pc2.create_cloud(header, fields, cloud_rows.tolist())

    def _publish_map_cloud(self) -> None:
        cloud_msg = self._voxel_map_to_pointcloud()
        if cloud_msg is None:
            return
        self._map_pub.publish(cloud_msg)

    def _pc_cb(self, msg: PointCloud2) -> None:
        self._frame_idx += 1
        if self._frame_idx % self._frame_skip != 0:
            return

        pts, rgb = _parse_xyz_rgb(msg)
        if pts is None or len(pts) == 0:
            return

        dist = np.linalg.norm(pts, axis=1)
        mask = dist < self._max_range
        pts, rgb = pts[mask], rgb[mask]
        if len(pts) == 0:
            return

        try:
            tf_s = self._tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.2),
            )
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
            return

        R, t = _tf_to_Rt(tf_s)
        pts_map = (R @ pts.T).T + t
        self._accumulate_pts(pts_map, rgb)

    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan_frame_idx += 1
        if self._scan_frame_idx % max(1, self._scan_frame_skip) != 0:
            return

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        if ranges.size == 0:
            return

        max_r = min(
            self._scan_max_range, msg.range_max if msg.range_max > 0.0 else self._scan_max_range
        )
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= max_r)
        if not np.any(valid):
            return

        idx = np.nonzero(valid)[0]
        r = ranges[valid]
        angles = msg.angle_min + idx * msg.angle_increment

        x = r * np.cos(angles)
        y = r * np.sin(angles)
        z = np.zeros_like(x)
        pts_scan = np.column_stack((x, y, z))

        try:
            scan_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            tf_s = self._tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                scan_stamp,
                rclpy.duration.Duration(seconds=0.2),
            )
        except Exception as e:
            self.get_logger().debug(f'Scan TF lookup failed: {e}')
            return

        R, t = _tf_to_Rt(tf_s)
        pts_map = (R @ pts_scan.T).T + t
        scan_rgb = np.tile(np.array(self._scan_rgb, dtype=np.uint8), (len(pts_map), 1))
        self._accumulate_pts(pts_map, scan_rgb)

    def _save(self) -> None:
        n = len(self._voxel_map)
        if n == 0:
            self.get_logger().warn('PlySaver: no voxels yet - is the camera publishing?')
            return

        vs = self._voxel_size
        items = list(self._voxel_map.items())
        keys = np.array([k for k, _ in items], dtype=np.int64)
        colors = np.array([v for _, v in items], dtype=np.uint8)
        pts = keys.astype(np.float64) * vs + vs / 2

        try:
            _write_ply_rgb(self._ply_path, pts, colors)
            self.get_logger().info(f'PlySaver: saved {n} pts -> {self._ply_path.name}')
        except OSError as exc:
            self.get_logger().error(f'PlySaver: save failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = PlySaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
