import pytest

pytest.importorskip('sensor_msgs_py')
import numpy as np
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from reseq_traction_autonomy.cloud_decimator import decimate


def organized_cloud(width, height):
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    points = []
    for row in range(height):
        for col in range(width):
            points.append((float(col), float(row), 1.0, 0.0))
    points[0] = (float('nan'), 0.0, 1.0, 0.0)
    cloud = point_cloud2.create_cloud(Header(), fields, points)
    cloud.width = width
    cloud.height = height
    cloud.row_step = width * cloud.point_step
    return cloud


def test_decimation_keeps_every_nth_row_and_column_and_drops_nan():
    xyz = decimate(organized_cloud(4, 4), 2)
    # Pixels (0,0), (2,0), (0,2), (2,2); the first one is NaN.
    assert xyz.tolist() == [[2.0, 0.0, 1.0], [0.0, 2.0, 1.0], [2.0, 2.0, 1.0]]
    assert xyz.dtype == np.float32
