import math
import time
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from reseq_traction_autonomy.core import Tasks
from reseq_traction_autonomy.manager import Manager
from reseq_traction_autonomy.models import foresight_waypoints


def plan(frame='odom', verdict=True):
    pose = SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame),
        path=SimpleNamespace(header=SimpleNamespace(frame_id=frame), poses=[pose]),
        verdict=SimpleNamespace(data=verdict),
        reason=SimpleNamespace(data='critic rejected'),
    )


def test_foresight_path_is_read_in_odometry_frame():
    assert foresight_waypoints(plan()) == [[1.0, 2.0, 0.0]]


@pytest.mark.parametrize('frame, verdict', [('map', True), ('base_link', True), ('odom', False)])
def test_foresight_rejects_relabelled_frames_and_negative_verdicts(frame, verdict):
    with pytest.raises(ValueError):
        foresight_waypoints(plan(frame, verdict))


def manager():
    tasks = Tasks(clock=lambda: 10)
    removed = []
    node = SimpleNamespace(
        tasks=tasks,
        nav_epoch=0,
        nav_pending=False,
        nav_goal=None,
        model_future=None,
        model_context=None,
        foresight=SimpleNamespace(remove_pending_request=removed.append),
        log_file=None,
        gate_reason='open',
        autonomy_enabled=False,
        enabled_generation=None,
        _event=lambda *args: None,
    )
    node._finish = lambda status, reason: Manager._finish(node, status, reason)
    return node, removed


def test_finished_task_drops_its_pending_model_request():
    node, removed = manager()
    node.tasks.start('visual', {}, 20)
    pending = SimpleNamespace(cancel=lambda: None)
    node.model_future = pending
    node.model_context = {'model': 'foresight'}
    Manager._finish(node, 'cancelled', 'test')
    assert node.model_future is None
    assert removed == [pending]


def test_late_disable_from_previous_task_does_not_cancel_unconfirmed_task():
    node, _ = manager()
    node.autonomy_enabled = True
    task = node.tasks.start('metric', {}, 20)
    Manager._enabled_cb(node, Bool(data=False))
    assert task.status == 'running'
    node.autonomy_enabled = True
    node.enabled_generation = node.tasks.generation
    Manager._enabled_cb(node, Bool(data=False))
    assert task.status == 'cancelled'


def test_gate_opens_only_with_goal_fresh_prediction_and_fresh_command():
    node, _ = manager()
    node.cfg = {'prediction_max_age_s': 6.0}
    node.last_candidate = Twist()
    node.last_candidate.linear.x = 0.1
    assert Manager._gate(node)[1] == 'idle'
    node.tasks.start('visual', {}, 20)
    node.autonomy_enabled = True
    node.nav_goal = object()
    node.last_prediction = time.monotonic() - 10
    node.candidate_time = time.monotonic()
    assert Manager._gate(node)[1] == 'prediction_expired'
    node.last_prediction = time.monotonic()
    command, reason = Manager._gate(node)
    assert reason == 'open'
    assert command.linear.x == 0.1
    node.candidate_time = time.monotonic() - 1
    assert Manager._gate(node)[1] == 'nav_command_expired'


def test_every_relative_waypoint_is_anchored_at_the_observation_pose():
    validated = []

    def validate(x, y, yaw, frame):
        validated.append(frame)
        return x, y, yaw

    node = SimpleNamespace(
        map_frame='map',
        cfg={'min_subgoal_distance_m': 0.12, 'max_subgoal_distance_m': 2.0},
        _pose=lambda frame: (0.0, 0.0, 0.0),
        _validate_goal=validate,
    )
    rows = [[0.3, 0.0, 0.0], [0.6, 0.0, 0.0], [3.0, 0.0, 0.0]]
    origin = {'x': 1.0, 'y': 0.0, 'yaw': math.pi / 2}
    x, y, yaw = Manager._choose_subgoal(node, rows, 'base_link', origin)
    assert (x, y, yaw) == pytest.approx((1.0, 0.6, math.pi / 2))
    assert validated == ['odom', 'odom']
