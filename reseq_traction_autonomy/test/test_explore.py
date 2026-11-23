import math

import pytest

from reseq_traction_autonomy.core import Grid
from reseq_traction_autonomy.explore import choose_frontier, find_frontiers, viewing_poses
from reseq_traction_autonomy.explore_check import explore
from reseq_traction_autonomy.parameters import DEFAULTS
from reseq_traction_autonomy.replay import world_grid

CFG = {
    'frontier_min_size_m': 0.5,
    'frontier_min_distance_m': 1.0,
    'frontier_size_weight': 0.5,
    'frontier_tried_radius_m': 1.0,
    'frontier_standoffs_m': [0.6, 0.9],
}


def half_known(width=40, height=20, known_cols=20):
    """A 4 x 2 m grid of 0.1 m cells: the left half free, the right half unknown."""
    data = []
    for _ in range(height):
        for col in range(width):
            if col < known_cols:
                data.append(0)
            else:
                data.append(-1)
    return Grid(width, height, 0.1, (0.0, 0.0, 0.0), data)


def anywhere(x, y, yaw):
    return x, y, yaw


def test_the_edge_of_the_known_area_is_one_frontier():
    frontiers = find_frontiers(half_known(), 0.5)
    assert len(frontiers) == 1
    x, y, size = frontiers[0]
    assert x == pytest.approx(1.95)
    assert size == pytest.approx(2.0)


def test_short_frontiers_are_ignored():
    assert find_frontiers(half_known(height=3), 0.5) == []


def test_viewing_poses_face_the_frontier_from_the_robot_side():
    poses = viewing_poses((2.0, 0.0), (0.0, 0.0, 0.0), [0.6, 0.9])
    assert poses[0] == pytest.approx((1.4, 0.0, 0.0))
    assert poses[1] == pytest.approx((1.1, 0.0, 0.0))


def test_choose_frontier_skips_frontiers_tried_before():
    grid = half_known()
    goal, frontier = choose_frontier(grid, (0.5, 1.0, 0.0), anywhere, [], CFG)
    assert goal[2] == pytest.approx(0.0, abs=0.1)
    assert frontier[0] == pytest.approx(1.95)
    goal, frontier = choose_frontier(grid, (0.5, 1.0, 0.0), anywhere, [frontier], CFG)
    assert goal is None and frontier is None


def test_frontiers_right_next_to_the_robot_are_skipped():
    # The frontier is at x 1.95; standing at x 1.5 it is too close to count.
    assert choose_frontier(half_known(), (1.5, 1.0, 0.0), anywhere, [], CFG) == (None, None)


def test_a_frontier_with_no_free_pose_is_passed_over():
    def nowhere(x, y, yaw):
        return None

    assert choose_frontier(half_known(), (0.5, 1.0, 0.0), nowhere, [], CFG) == (None, None)


# Two 3 x 3 m rooms side by side, joined by a 1.2 m door. The target is in the right room.
TWO_ROOMS = """<sdf version="1.9"><world name="rooms">
  <model name="north"><pose>1.5 1.5 0 0 0 0</pose><link name="link"><collision name="c">
    <geometry><box><size>6.2 0.1 1</size></box></geometry></collision></link></model>
  <model name="south"><pose>1.5 -1.5 0 0 0 0</pose><link name="link"><collision name="c">
    <geometry><box><size>6.2 0.1 1</size></box></geometry></collision></link></model>
  <model name="west"><pose>-1.5 0 0 0 0 1.5708</pose><link name="link"><collision name="c">
    <geometry><box><size>3.1 0.1 1</size></box></geometry></collision></link></model>
  <model name="east"><pose>4.5 0 0 0 0 1.5708</pose><link name="link"><collision name="c">
    <geometry><box><size>3.1 0.1 1</size></box></geometry></collision></link></model>
  <model name="middle_low"><pose>1.5 -1.05 0 0 0 1.5708</pose><link name="link"><collision name="c">
    <geometry><box><size>0.9 0.1 1</size></box></geometry></collision></link></model>
  <model name="middle_high"><pose>1.5 1.05 0 0 0 1.5708</pose><link name="link"><collision name="c">
    <geometry><box><size>0.9 0.1 1</size></box></geometry></collision></link></model>
</world></sdf>"""


def test_exploration_reaches_the_other_room_and_sees_the_target(tmp_path):
    path = tmp_path / 'rooms.world'
    path.write_text(TWO_ROOMS)
    steps, target_seen_at = explore(world_grid(path), DEFAULTS, target=(3.5, 0.0))
    assert target_seen_at is not None
    assert steps[-1]['explored'] > 0.9
    assert math.isfinite(steps[-1]['travelled_m'])
