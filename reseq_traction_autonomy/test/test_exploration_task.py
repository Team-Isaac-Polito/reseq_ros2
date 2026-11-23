import math
import time
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')

from reseq_traction_autonomy.core import Tasks
from reseq_traction_autonomy.manager import Manager


def exploring_node():
    """A stub manager running an explore task, with Nav2 and the map replaced."""
    node = SimpleNamespace(
        tasks=Tasks(clock=lambda: 10),
        cfg={
            'explore_goal_timeout_s': 20.0,
            'explore_progress_m': 0.2,
            'explore_done_after_s': 10.0,
        },
        grid=None,
        last_area_update=0.0,
        nav_pending=False,
        nav_goal=None,
        backing_up=False,
        no_frontier_since=None,
        best_remaining=math.inf,
        last_progress=0.0,
        map_frame='map',
        nav_epoch=0,
        autonomy_enabled=False,
        gate_reason='open',
        _event=lambda *args: None,
    )
    for name in ('_finish', '_cancel_model_request', '_making_progress', '_drive_exploration'):
        setattr(node, name, getattr(Manager, name).__get__(node))
    node.model_future = None
    node.model_context = None
    node.log_file = None
    sent = []
    node._send_goal = lambda x, y, yaw, frame, replace=False: sent.append((x, y))
    task = node.tasks.start('explore', {}, 300)
    return node, task, sent


def test_exploration_drives_to_the_next_frontier():
    node, task, sent = exploring_node()
    node._frontier_goal = lambda: (2.0, 1.0, 0.0)
    node._drive_exploration(task)
    assert sent == [(2.0, 1.0)]
    assert task.progress['frontiers'] == 1
    assert node.subgoal_kind == 'frontier'


def test_a_long_drive_is_kept_while_the_distance_keeps_shrinking():
    node, task, sent = exploring_node()
    node._frontier_goal = lambda: (7.0, 0.0, 0.0)
    node._drive_exploration(task)
    node.nav_goal = object()
    # 30 s later the robot is still closing in, so the goal is kept.
    now = time.monotonic()
    node.last_progress = now - 15.0
    node.best_remaining = 3.0
    task.progress['distance_remaining_m'] = 2.0
    node._drive_exploration(task)
    assert sent == [(7.0, 0.0)]


def test_a_goal_without_progress_is_given_up_for_the_next_frontier():
    node, task, sent = exploring_node()
    node._frontier_goal = lambda: (7.0, 0.0, 0.0)
    node._drive_exploration(task)
    node.nav_goal = object()
    node.last_progress = time.monotonic() - 21.0
    node.best_remaining = 3.0
    task.progress['distance_remaining_m'] = 2.95
    node._frontier_goal = lambda: (1.0, 2.0, 0.0)
    node._drive_exploration(task)
    assert sent == [(7.0, 0.0), (1.0, 2.0)]


def test_exploration_finishes_after_a_quiet_while_without_frontiers():
    node, task, sent = exploring_node()
    node._frontier_goal = lambda: None
    node._drive_exploration(task)
    assert task.status == 'running'
    node.no_frontier_since = time.monotonic() - 11.0
    node._drive_exploration(task)
    assert task.status == 'succeeded'
    assert sent == []
