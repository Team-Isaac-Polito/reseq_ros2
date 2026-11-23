from collections import deque
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from sensor_msgs.msg import Image
from tf2_ros import TransformException

from reseq_traction_autonomy.ros_interface import RosInterface


def image(nanosec):
    message = Image()
    message.header.stamp.sec = 10
    message.header.stamp.nanosec = nanosec
    message.header.frame_id = 'camera'
    return message


@pytest.fixture
def camera(monkeypatch):
    monkeypatch.setattr('reseq_traction_autonomy.ros_interface.time.monotonic', lambda: 100.0)
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_300_000_000))
    node = SimpleNamespace(
        images=deque(),
        cfg={'camera_max_age_s': 2.0},
        get_clock=lambda: clock,
        _jpeg=lambda message: (b'jpeg', (640, 480)),
    )
    node._recent_image = lambda: RosInterface._recent_image(node)
    return node


def test_camera_uses_recent_frame_with_transforms_at_its_own_timestamp(camera):
    camera.images.extend([(image(100_000_000), 99.8), (image(300_000_000), 99.9)])
    lookups = []

    def pose(frame, stamp, source='base_link'):
        lookups.append((frame, stamp.nanoseconds, source))
        if stamp.nanoseconds > 10_200_000_000:
            raise TransformException('Transform has not arrived yet')
        return {'x': 1.0, 'y': 2.0, 'yaw': 0.0}

    camera._pose_json = pose
    observation = RosInterface._observation(camera)
    assert observation['stamp_s'] == pytest.approx(10.1)
    assert observation['age_s'] == pytest.approx(0.2)
    assert lookups[-2:] == [
        ('odom', 10_100_000_000, 'base_link'),
        ('odom', 10_100_000_000, 'camera'),
    ]


def test_camera_rejects_frames_without_synchronized_transforms(camera):
    camera.images.append((image(100_000_000), 99.9))

    def missing(*args):
        raise TransformException('Missing transform')

    camera._pose_json = missing
    with pytest.raises(ValueError, match='matching transforms'):
        RosInterface._observation(camera)


def test_camera_expires_in_wall_time_when_simulation_clock_stops(camera):
    camera.images.append((image(100_000_000), 97.0))
    with pytest.raises(ValueError, match='No recent camera image'):
        RosInterface._observation(camera)
