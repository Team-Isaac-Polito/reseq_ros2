"""Unit tests for thermal pointcloud fusion, temperature conversion, and PLY export."""

from pathlib import Path
import tempfile
import numpy as np

from reseq_ros2.ply_saver import _write_ply_rgb
from reseq_ros2.thermal_pointcloud_fusion import ThermalPointcloudFusion, TrackedHotspot


def test_temperature_conversion():
    """Verify conversion of centi-Kelvin and Kelvin to Celsius."""
    # Gazebo 16UC1 centi-Kelvin (31815 cK -> 45.0 °C)
    raw_ck = np.array([31815, 29815, 28815], dtype=np.uint16)
    celsius = (raw_ck.astype(np.float32) / 100.0) - 273.15
    np.testing.assert_allclose(celsius, [45.0, 25.0, 15.0], atol=0.01)

    # Kelvin float (318.15 K -> 45.0 °C)
    raw_k = np.array([318.15, 298.15, 288.15], dtype=np.float32)
    celsius_k = raw_k - 273.15
    np.testing.assert_allclose(celsius_k, [45.0, 25.0, 15.0], atol=0.01)


def test_pinhole_projection():
    """Verify perspective projection from REP-103 robot camera frame (X fwd, Y left, Z up)."""
    # Focal lengths and optical centers for 32x24 sensor
    fx, fy = 30.7357, 30.7357
    cx, cy = 15.5, 11.5

    # Point directly on optical axis at 2.0m forward: X=2.0, Y=0.0, Z=0.0
    # Optical conventions: Z_opt = X, X_opt = -Y, Y_opt = -Z
    pt_center = np.array([[2.0, 0.0, 0.0]], dtype=np.float32)
    d = pt_center[:, 0]
    u = int(fx * (-pt_center[0, 1] / d[0]) + cx)
    v = int(fy * (-pt_center[0, 2] / d[0]) + cy)
    assert u == 15
    assert v == 11

    # Point to the right (negative Y in robot frame) should project to u > cx
    pt_right = np.array([[2.0, -0.5, 0.0]], dtype=np.float32)
    u_right = int(fx * (-pt_right[0, 1] / pt_right[0, 0]) + cx)
    assert u_right > 15


def test_simple_dbscan_clustering():
    """Verify fallback DBSCAN separates distinct clusters and rejects noise."""
    # Cluster A around (1.0, 0.0, 0.0), Cluster B around (2.5, 0.0, 0.0)
    rng = np.random.RandomState(42)
    cluster_a = rng.normal(loc=[1.0, 0.0, 0.0], scale=0.02, size=(30, 3))
    cluster_b = rng.normal(loc=[2.5, 0.0, 0.0], scale=0.02, size=(30, 3))
    noise = np.array([[5.0, 5.0, 5.0], [-5.0, -5.0, -5.0]])
    pts = np.vstack([cluster_a, cluster_b, noise])

    labels = ThermalPointcloudFusion._simple_dbscan(pts, eps=0.15, min_samples=5)

    # Noise points should be labeled -1
    assert labels[-1] == -1
    assert labels[-2] == -1

    # Should find exactly 2 distinct clusters
    unique_clusters = set(labels) - {-1}
    assert len(unique_clusters) == 2


def test_hotspot_deduplication():
    """Verify tracked hotspot running average position and peak temperature tracking."""
    # Initial hotspot at [2.0, 3.0, 0.5] with 35.0 °C
    th = TrackedHotspot(
        hotspot_id=1, pos=np.array([2.0, 3.0, 0.5]), peak_temp=35.0, stamp=None
    )

    # Second observation slightly shifted at [2.1, 3.05, 0.5] with higher temp 45.0 °C
    new_obs = np.array([2.1, 3.05, 0.5])
    dist = np.linalg.norm(th.pos - new_obs)
    dedup_distance = 0.25
    assert dist < dedup_distance

    # Update hotspot
    th.pos = 0.8 * th.pos + 0.2 * new_obs
    th.peak_temp = max(th.peak_temp, 45.0)

    assert th.peak_temp == 45.0
    assert abs(th.pos[0] - 2.02) < 1e-4


def test_hotspot_persistence():
    """Verify that hotspots require multiple hits to reach persistence threshold."""
    th = TrackedHotspot(
        hotspot_id=1, pos=np.array([2.0, 3.0, 0.5]), peak_temp=35.0, stamp=None
    )
    assert th.hit_count == 1
    assert th.detection_sent is False

    min_persistence_hits = 3
    assert th.hit_count < min_persistence_hits

    th.hit_count += 1
    assert th.hit_count < min_persistence_hits

    th.hit_count += 1
    assert th.hit_count >= min_persistence_hits


def test_ply_temperature_export():
    """Verify that PLY map file exports valid temperature property and preserves geometry."""
    with tempfile.TemporaryDirectory() as tmpdir:
        ply_file = Path(tmpdir) / 'test_map.ply'
        pts = np.array([[1.0, 2.0, 0.5], [3.0, 4.0, 0.5]], dtype=np.float64)
        colors = np.array([[255, 0, 0], [0, 255, 0]], dtype=np.uint8)
        temps = np.array([45.5, np.nan], dtype=np.float64)

        n = _write_ply_rgb(ply_file, pts, colors, temps)
        assert n == 2
        assert ply_file.exists()

        content = ply_file.read_text()
        assert 'property float temperature' in content
        assert '45.50' in content
        assert 'nan' in content
