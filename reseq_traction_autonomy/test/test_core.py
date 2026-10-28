import math
import pytest

from reseq_traction_autonomy.core import Grid, Tasks, convex_hull, finite, transform_xy


def test_cancelled_request_generation_cannot_control_a_new_task():
    tasks = Tasks(clock=lambda: 10)
    first = tasks.start('visual', {}, 20)
    old_generation = tasks.generation
    tasks.finish('cancelled', 'manual_override')
    second = tasks.start('metric', {}, 30)
    assert first.status == 'cancelled'
    assert second.id != first.id
    assert not tasks.current(first.id, old_generation)


def test_terminal_status_does_not_keep_elapsed_time_growing():
    now = [1.0]
    tasks = Tasks(clock=lambda: now[0])
    task = tasks.start('metric', {}, 5)
    now[0] = 3
    tasks.finish('succeeded')
    now[0] = 50
    assert task.view(now[0])['elapsed_s'] == 2


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf'), True])
def test_non_finite_commands_are_rejected(value):
    with pytest.raises(ValueError):
        finite(value, 'x')


def test_busy_task_cannot_be_silently_replaced():
    tasks = Tasks(clock=lambda: 10)
    tasks.start('metric', {}, 20)
    with pytest.raises(ValueError):
        tasks.start('visual', {}, 20)


def test_deadlines_use_elapsed_wall_time():
    now = [10.0]
    tasks = Tasks(clock=lambda: now[0])
    tasks.start('metric', {}, 5)
    now[0] = 16
    assert tasks.expired()


def test_relative_waypoint_is_anchored_at_observation_pose():
    assert transform_xy((1, 0), (4, 5, math.pi / 2)) == pytest.approx((4, 6))


def test_rotated_map_origin_is_respected():
    grid = Grid(10, 10, 1.0, (5, 5, math.pi / 2), [0] * 100)
    assert grid.cell(4.5, 5.5) == (0, 0)
    assert grid.cell(5.5, 4.5) is None


def test_obstacle_inside_footprint_is_not_missed_when_vertices_are_free():
    data = [0] * 100
    data[5 * 10 + 5] = 100
    grid = Grid(10, 10, 1.0, (0, 0, 0), data)
    footprint = [(-2, -2), (2, -2), (2, 2), (-2, 2)]
    assert not grid.pose_free((5.5, 5.5, 0), footprint)


def test_unknown_and_out_of_map_goals_are_rejected():
    grid = Grid(2, 2, 1.0, (0, 0, 0), [-1, 0, 0, 0])
    assert not grid.traversable(0.5, 0.5)
    assert not grid.traversable(-0.1, 0.5)
    assert grid.traversable(1.5, 1.5)


def test_bent_body_footprint_contains_all_module_corners():
    points = [(0, 0), (1, 0), (1, 1), (0, 1), (-1, -1)]
    hull = convex_hull(points)
    assert (-1, -1) in hull and (1, 1) in hull
    assert (0, 0) not in hull
