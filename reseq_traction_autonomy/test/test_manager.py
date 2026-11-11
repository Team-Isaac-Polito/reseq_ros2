import math
import time
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from reseq_traction_autonomy.core import Tasks
from reseq_traction_autonomy.manager import Manager
from reseq_traction_autonomy.navigation import Navigation
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


def bind(node, cls, *names):
    """Attach methods of cls to a stub node, so methods can call each other."""
    for name in names:
        method = getattr(cls, name)
        setattr(node, name, method.__get__(node))


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
    bind(
        node,
        Manager,
        '_finish',
        '_cancel_model_request',
        '_read_prediction',
        '_stop_claimed',
        '_metric_goal_finished',
    )
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


def test_gate_flips_nav2_turns_into_the_traction_convention_and_clamps():
    node = SimpleNamespace(
        cfg={'max_linear_speed': 0.19, 'max_angular_speed': 1.0, 'traction_turn_sign': -1.0}
    )
    message = Twist()
    message.linear.x = 0.5
    message.angular.z = 0.4
    Navigation._candidate_cb(node, message)
    assert node.last_candidate.linear.x == pytest.approx(0.19)
    assert node.last_candidate.angular.z == pytest.approx(-0.4)
    message.angular.z = -3.0
    Navigation._candidate_cb(node, message)
    assert node.last_candidate.angular.z == pytest.approx(1.0)


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


def test_only_a_run_of_stop_claims_ends_a_visual_task():
    node, _ = manager()
    node.cfg = {'stop_confirmations': 3}
    node.stop_claims = 0
    node.last_prediction = None
    node.nav_pending = True  # a Nav2 goal is on its way, so moves send nothing new
    node._in_evaluation_region = lambda task: None
    task = node.tasks.start('visual', {}, 20)
    context = {'model': 'lightnav', 'observation': {'stamp_s': 1.0}}
    stop = {'frame': 'base_link', 'waypoints': [], 'stop': True}
    move = {'frame': 'base_link', 'waypoints': [[0.3, 0.0, 0.0]], 'stop': False}
    for result in (stop, stop, move, stop, stop):
        Manager._apply_prediction(node, result, context)
    assert task.status == 'running'
    Manager._apply_prediction(node, stop, context)
    assert task.status == 'succeeded'
    assert task.reason == 'Model stop claim; destination not measured'


def test_prediction_without_a_feasible_subgoal_keeps_the_task_running():
    node, _ = manager()
    node.cfg = {'stop_confirmations': 3}
    node.stop_claims = 0
    node.last_prediction = None
    events = []
    node._event = lambda kind, data: events.append(kind)
    node._choose_subgoal = lambda rows, frame, pose: None
    node.backing_up = False
    node._note_unreachable = lambda: events.append('unreachable')
    task = node.tasks.start('visual', {}, 20)
    context = {'model': 'lightnav', 'observation': {'stamp_s': 1.0, 'robot_pose': {}}}
    turn_in_place = {'frame': 'base_link', 'waypoints': [[0.0, 0.0, -0.31]], 'stop': False}
    Manager._apply_prediction(node, turn_in_place, context)
    assert task.status == 'running'
    assert events[-2:] == ['no_feasible_subgoal', 'unreachable']


def visual_node(subgoal_age_s):
    node, _ = manager()
    sent = []
    node.cfg = {'stop_confirmations': 3, 'subgoal_refresh_s': 3.0}
    node.stop_claims = 0
    node.last_prediction = None
    node.backing_up = False
    node.nav_goal = object()  # Nav2 is driving an earlier subgoal
    node.subgoal_sent = time.monotonic() - subgoal_age_s
    node.unreachable_predictions = 0
    node.map_frame = 'map'
    node._choose_subgoal = lambda rows, frame, pose: (1.0, 0.0, 0.0)
    node._send_goal = lambda x, y, yaw, frame, replace=False: sent.append(replace)
    node.tasks.start('visual', {}, 20)
    return node, sent


def reversing_node(tail_pose):
    """A node driving a goal at (0.15, 0, yaw 0) whose tail is at tail_pose in the map."""
    tasks = Tasks(clock=lambda: 10)
    task = tasks.start('metric', {}, 20)
    task.subgoal = {'x': 0.15, 'y': 0.0, 'yaw': 0.0, 'frame': 'map'}
    node = SimpleNamespace(
        tasks=tasks,
        nav_goal=object(),
        backing_up=False,
        map_frame='map',
        cfg={
            'num_modules': 3,
            'reverse_lookahead_m': 0.6,
            'max_linear_speed': 0.19,
            'max_angular_speed': 1.0,
            'traction_turn_sign': -1.0,
        },
        _pose=lambda frame, source: tail_pose,
    )
    bind(node, Navigation, '_reverse_turn_rate')
    return node


def test_newer_prediction_replaces_a_subgoal_only_after_the_refresh_time():
    context = {'model': 'lightnav', 'observation': {'stamp_s': 1.0, 'robot_pose': {}}}
    move = {'frame': 'base_link', 'waypoints': [[0.6, 0.0, 0.0]], 'stop': False}
    young, young_sent = visual_node(subgoal_age_s=1.0)
    Manager._apply_prediction(young, move, context)
    assert young_sent == []
    old, old_sent = visual_node(subgoal_age_s=5.0)
    Manager._apply_prediction(old, move, context)
    assert old_sent == [True]


def test_a_run_of_unreachable_predictions_backs_the_robot_up_once():
    backups = []
    node = SimpleNamespace(
        unreachable_predictions=0,
        cfg={'escape_after_unreachable': 2},
        _back_up=lambda: backups.append(1),
    )
    Navigation._note_unreachable(node)
    assert backups == []
    Navigation._note_unreachable(node)
    assert backups == [1]
    assert node.unreachable_predictions == 0


def test_failed_subgoal_continues_a_visual_task_but_fails_a_metric_task():
    for kind, expected in (('visual', 'running'), ('metric', 'failed')):
        node, _ = manager()
        events = []
        node._event = lambda name, data: events.append(name)
        node._note_unreachable = lambda: events.append('unreachable')
        node.next_prediction = 5.0
        task = node.tasks.start(kind, {}, 20)
        Navigation._goal_finished(node, GoalStatus.STATUS_ABORTED)
        assert task.status == expected
        if kind == 'visual':
            assert events == ['subgoal_failed', 'unreachable']
            assert node.next_prediction == 0


def zone_task(deadline):
    """A metric task that has arrived and is waiting to read its zone QR code."""
    node, _ = manager()
    node.cfg = {'zone_qr_timeout_s': 3.0}
    node.qr_deadline = deadline
    task = node.tasks.start('metric', {'zone_qr': 'END'}, 20)
    return node, task


def test_arriving_with_a_zone_qr_waits_for_the_code_before_finishing():
    node, task = zone_task(None)
    node.qr_text = 'END'
    Navigation._goal_finished(node, GoalStatus.STATUS_SUCCEEDED)
    assert node.qr_text == ''
    assert task.status == 'running'
    assert node.qr_deadline is not None


@pytest.mark.parametrize(
    'text, deadline, expected',
    [('END', math.inf, 'succeeded'), ('', math.inf, 'running'), ('', 0.0, 'failed')],
)
def test_zone_qr_check_succeeds_on_the_right_text_and_fails_after_timeout(
    text, deadline, expected
):
    node, task = zone_task(deadline)
    node.qr_text = text
    Navigation._check_zone_qr(node)
    assert task.status == expected


def test_reversing_tail_on_the_goal_line_goes_straight():
    node = reversing_node((1.0, 0.0, 0.0))
    assert node._reverse_turn_rate(-0.19, 0.3) == pytest.approx(0.0)


def test_reversing_tail_steers_back_to_the_goal_line():
    left = reversing_node((1.0, 0.2, 0.0))
    right = reversing_node((1.0, -0.2, 0.0))
    assert left._reverse_turn_rate(-0.19, 0.0) > 0
    assert right._reverse_turn_rate(-0.19, 0.0) < 0


def test_reverse_steering_is_left_to_nav2_when_it_does_not_apply():
    node = reversing_node((1.0, 0.2, 0.0))
    node.backing_up = True
    assert node._reverse_turn_rate(-0.19, 0.3) == 0.3

    node = reversing_node((1.0, 0.2, 0.0))
    node.nav_goal = None
    assert node._reverse_turn_rate(-0.19, 0.3) == 0.3

    node = reversing_node((1.0, 0.2, 0.0))
    node.tasks.active.subgoal = None
    assert node._reverse_turn_rate(-0.19, 0.3) == 0.3

    node = reversing_node((1.0, 0.2, math.pi / 2))
    assert node._reverse_turn_rate(-0.19, 0.3) == 0.3


def test_reverse_command_is_steered_for_the_tail_and_sign_flipped():
    node = reversing_node((1.0, 0.2, 0.0))
    message = Twist()
    message.linear.x = -0.19
    message.angular.z = 0.0
    Navigation._candidate_cb(node, message)
    assert node.last_candidate.linear.x == pytest.approx(-0.19)
    assert node.last_candidate.angular.z < 0
