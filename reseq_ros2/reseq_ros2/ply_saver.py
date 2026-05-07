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

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2


def _parse_xyz_rgb(msg: PointCloud2):
    """Extract XYZ (float32) and RGB (uint8) from a colored PointCloud2.

    The RGB field may be stored as a single float32 (Gazebo bridge packs
    RGBA into one float) or as separate uint8 values (RealSense native).
    Returns (pts_xyz (N,3) float64, pts_rgb (N,3) uint8) or (None, None).
    """
    fields = {f.name: (f.offset, f.datatype) for f in msg.fields}
    if not all(k in fields for k in ('x', 'y', 'z', 'rgb')):
        return None, None

    ox, oy, oz = fields['x'][0], fields['y'][0], fields['z'][0]
    o_rgb, rgb_dt = fields['rgb']
    step = msg.point_step
    n = msg.width * msg.height

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)

    xyz = raw[:, ox:ox + 12].copy().view(np.float32).reshape(n, 3).astype(np.float64)

    if rgb_dt == 7:  # FLOAT32 - Gazebo bridge packs RGBA into one float32
        rgb = raw[:, o_rgb:o_rgb + 4].astype(np.uint8)[:, :3].copy()
    else:  # UINT8, UINT32, etc. - RealSense native format
        rgb = raw[:, o_rgb:o_rgb + 3].copy()

    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid], rgb[valid]


def _tf_to_Rt(tf_stamped):
    """Return (R 3x3 float64, t 3-vec float64) from a TF stamped transform."""
    tr = tf_stamped.transform.translation
    q = tf_stamped.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)
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
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('frame_skip', 5)
        self.declare_parameter('max_range', 10.0)

        self._pc_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        save_dir = Path(self.get_parameter('save_path').get_parameter_value().string_value)
        interval = self.get_parameter('save_interval').get_parameter_value().double_value
        self._voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self._frame_skip = self.get_parameter('frame_skip').get_parameter_value().integer_value
        self._max_range = self.get_parameter('max_range').get_parameter_value().double_value

        self._ply_path = save_dir / 'reseq_map_3d.ply'
        self._voxel_map = {}
        self._frame_idx: int = 0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            PointCloud2, self._pc_topic, self._pc_cb, qos,
        )

        self.create_timer(interval, self._save)
        self.get_logger().info(
            f'PlySaver listening on [{self._pc_topic}], '
            f'save every {interval:.0f}s -> {self._ply_path}'
        )

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
                'map', msg.header.frame_id,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.2),
            )
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
            return

        R, t = _tf_to_Rt(tf_s)
        pts_map = (R @ pts.T).T + t

        vs = self._voxel_size
        voxel_keys = tuple(np.floor(pts_map[:, i] / vs).astype(np.int64) for i in range(3))
        
        # Update voxel map (last observation wins per voxel)
        for k0, k1, k2, c in zip(*voxel_keys, rgb):
            self._voxel_map[(int(k0), int(k1), int(k2))] = (int(c[0]), int(c[1]), int(c[2]))

        if self._frame_idx % (self._frame_skip * 30) == 0:
            self.get_logger().info(
                f'PlySaver: {len(self._voxel_map)} voxels accumulated'
            )

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
            self.get_logger().info(
                f'PlySaver: saved {n} pts -> {self._ply_path.name}'
            )
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
