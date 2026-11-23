import math

import pytest

from reseq_traction_autonomy.subgoals import (
    FORWARD_ARC,
    REVERSE_ARC,
    WAYPOINT,
    arc_end,
    choose_subgoal,
    requested_turn,
    turn_arcs,
    wrap_angle,
)

LIMITS = {
    'min_subgoal_distance_m': 0.12,
    'max_subgoal_distance_m': 2.0,
    'turn_arcs': True,
    'min_turn_rad': 0.15,
    'turn_arc_length_m': 0.8,
    'turn_arc_max_rad': 0.8,
}
ORIGIN = (0.0, 0.0, 0.0)


def anywhere(x, y, yaw, frame):
    return x, y, yaw


def only_behind(x, y, yaw, frame):
    """A map where only poses behind the robot are free."""
    if x < 0:
        return x, y, yaw
    return None


def test_wrap_angle_keeps_turns_short():
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert wrap_angle(-0.3) == pytest.approx(-0.3)


def test_a_turn_on_the_spot_is_recognised_by_its_final_heading():
    turn = [(0.0, 0.0, -0.31), (0.0001, 0.0, -0.63)]
    assert requested_turn(turn, 0.12) == pytest.approx(-0.63)
    moving = [(0.0, 0.0, 0.2), (0.5, 0.0, 0.4)]
    assert requested_turn(moving, 0.12) is None
    assert requested_turn([], 0.12) is None


def test_forward_arc_ends_ahead_and_to_the_side_of_the_turn():
    x, y, yaw = arc_end(0.8, 0.8)
    assert x > 0 and y > 0
    assert yaw == pytest.approx(0.8)
    # The arc is 0.8 m long, so its chord is a little shorter.
    assert math.hypot(x, y) == pytest.approx(2 * (0.8 / 0.8) * math.sin(0.4))


def test_reverse_arc_ends_behind_and_turns_the_nose_the_same_way():
    x, y, yaw = arc_end(-0.8, 0.8)
    assert x < 0 and y < 0
    assert yaw == pytest.approx(0.8)


def test_a_straight_arc_has_no_division_by_zero():
    assert arc_end(0.8, 0.0) == (0.8, 0.0, 0.0)


def test_large_turns_are_limited_to_the_maximum_arc_angle():
    arcs = turn_arcs(-2.6, 0.8, 0.8)
    assert [kind for kind, _ in arcs] == [FORWARD_ARC, REVERSE_ARC]
    for _, pose in arcs:
        assert pose[2] == pytest.approx(-0.8)


def test_the_farthest_free_waypoint_wins():
    rows = [[0.3, 0.0, 0.0], [0.6, 0.0, 0.0], [3.0, 0.0, 0.0]]
    goal, kind = choose_subgoal(rows, 'base_link', ORIGIN, ORIGIN, anywhere, LIMITS)
    assert kind == WAYPOINT
    assert goal == pytest.approx((0.6, 0.0, 0.0))


def test_turn_on_the_spot_becomes_a_forward_arc():
    rows = [[0.0, 0.0, 0.5]]
    goal, kind = choose_subgoal(rows, 'base_link', ORIGIN, ORIGIN, anywhere, LIMITS)
    assert kind == FORWARD_ARC
    assert goal == pytest.approx(arc_end(0.8, 0.5))


def test_turn_on_the_spot_reverses_when_there_is_no_room_ahead():
    rows = [[0.0, 0.0, 0.5]]
    goal, kind = choose_subgoal(rows, 'base_link', ORIGIN, ORIGIN, only_behind, LIMITS)
    assert kind == REVERSE_ARC
    assert goal == pytest.approx(arc_end(-0.8, 0.5))


def test_arcs_start_at_the_pose_where_the_image_was_taken():
    origin = (1.0, 2.0, math.pi / 2)
    goal, _ = choose_subgoal([[0.0, 0.0, 0.5]], 'base_link', origin, origin, anywhere, LIMITS)
    x, y, yaw = arc_end(0.8, 0.5)
    # Facing +y, the arc's forward x becomes +y and its left y becomes -x.
    assert goal == pytest.approx((1.0 - y, 2.0 + x, math.pi / 2 + yaw))


def test_tiny_turns_absolute_frames_and_disabled_arcs_give_nothing():
    small = [[0.0, 0.0, 0.05]]
    assert choose_subgoal(small, 'base_link', ORIGIN, ORIGIN, anywhere, LIMITS) == (None, None)
    turn = [[0.0, 0.0, 0.5]]
    # Foresight plans in odom; it never asks for a turn on the spot.
    assert choose_subgoal(turn, 'odom', None, ORIGIN, anywhere, LIMITS) == (None, None)
    disabled = dict(LIMITS, turn_arcs=False)
    assert choose_subgoal(turn, 'base_link', ORIGIN, ORIGIN, anywhere, disabled) == (None, None)


def test_bad_waypoints_are_rejected():
    with pytest.raises(ValueError):
        choose_subgoal([[1.0]], 'base_link', ORIGIN, ORIGIN, anywhere, LIMITS)
    with pytest.raises(ValueError):
        choose_subgoal([[math.nan, 0.0]], 'base_link', ORIGIN, ORIGIN, anywhere, LIMITS)
