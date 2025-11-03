# compute_coordinate.py
import math
from typing import Optional, Tuple

import rclpy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo


def _get_time_from_detection(detection) -> Time:
    # detection may have builtin_interfaces/Time 'time' (original) or header
    if hasattr(detection, 'time'):
        # builtin_interfaces/Time
        return Time(seconds=int(detection.time.sec), nanoseconds=int(detection.time.nanosec))
    if hasattr(detection, 'header') and hasattr(detection.header, 'stamp'):
        return Time(
            seconds=int(detection.header.stamp.sec),
            nanoseconds=int(detection.header.stamp.nanosec),
        )
    return Time()  # now


def compute_coordinate(
    detection,
    camera_info: Optional[CameraInfo],
    target_frame: str,
    tf_buffer: tf2_ros.Buffer,
    tf_timeout_sec: float,
    fallback_intrinsics: dict,
) -> Tuple[bool, Optional[PointStamped], str]:
    """
    Returns (success, point_in_target_frame, message)
    """
    # Validate bbox and depth
    try:
        xmin = float(detection.xmin)
        ymin = float(detection.ymin)
        width = float(detection.width)
        height = float(detection.height)
        depth = float(detection.depth_center)
    except Exception as e:
        return False, None, f'invalid bbox/depth fields: {e}'

    if not math.isfinite(depth) or depth <= 0.0:
        return False, None, 'invalid depth_center'

    u = xmin + width / 2.0
    v = ymin + height / 2.0

    # get camera intrinsics
    # Use camera_info ONLY if it's provided and contains a non-zero focal length (k[0])
    if camera_info is not None and hasattr(camera_info, 'k') and camera_info.k[0] != 0.0:
        # from camera_info message
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]
    else:
        # from fallback
        fx = fallback_intrinsics.get('f_x', 0.0)
        fy = fallback_intrinsics.get('f_y', 0.0)
        cx = fallback_intrinsics.get('c_x', 0.0)
        cy = fallback_intrinsics.get('c_y', 0.0)

    # deproject
    x_cam = (u - cx) * depth / fx
    y_cam = (v - cy) * depth / fy
    z_cam = depth

    # Compose PointStamped in camera frame
    ps = PointStamped()
    ps.point.x = x_cam
    ps.point.y = y_cam
    ps.point.z = z_cam

    source_frame = (
        getattr(detection, 'camera_frame', None) or getattr(detection, 'frame_id', None) or ''
    )
    if not source_frame:
        return False, None, 'missing camera_frame in detection'

    ps.header.frame_id = source_frame
    # timestamp
    ps.header.stamp = _get_time_from_detection(detection).to_msg()

    # tf lookup and transform
    # first attempt: time-aware lookup
    try:
        timeout = Duration(seconds=tf_timeout_sec)
        transform = tf_buffer.lookup_transform(
            target_frame, source_frame, rclpy.time.Time.from_msg(ps.header.stamp), timeout=timeout
        )
        point_transformed = tf2_geometry_msgs.do_transform_point(ps, transform)
        return True, point_transformed, ''
    except Exception as e1:
        # retry once with latest transform
        try:
            transform = tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=tf_timeout_sec),
            )
            point_transformed = tf2_geometry_msgs.do_transform_point(ps, transform)
            return True, point_transformed, 'used latest transform (time-specific unavailable)'
        except Exception as e2:
            return False, None, f'tf lookup failed: {e1}; retry failed: {e2}'
