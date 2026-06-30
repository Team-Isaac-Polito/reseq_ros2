import pytest

from reseq_interfaces.msg import Remote
from reseq_ros2.scaler import _remote_axis_value


def test_remote_axis_value_reads_signed_axes():
    msg = Remote()
    msg.left.x = 0.25
    msg.left.y = -0.5
    msg.left.z = -0.75
    msg.right.z = 0.4

    assert _remote_axis_value(msg, 'left.x') == 0.25
    assert _remote_axis_value(msg, '-left.y') == 0.5
    assert _remote_axis_value(msg, '+right.z') == 0.4
    assert _remote_axis_value(msg, '-left.z') == 0.75


def test_remote_axis_value_rejects_invalid_specs():
    msg = Remote()

    with pytest.raises(ValueError):
        _remote_axis_value(msg, 'left.pitch')

    with pytest.raises(ValueError):
        _remote_axis_value(msg, 'middle.x')
