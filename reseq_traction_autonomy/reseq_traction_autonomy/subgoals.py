"""Choose the next subgoal from a navigation model's waypoints, independent of ROS.

The model answers with a few waypoints. The farthest one the robot can stand on
becomes the next Nav2 goal. Rese.Q cannot turn on the spot, so when the model
only asks to turn, a short arc that turns the same way is used instead.
"""

import math

from .core import clamp, finite, transform_xy

# Where a subgoal came from, as written in the task record.
FORWARD_ARC = 'forward_arc'
REVERSE_ARC = 'reverse_arc'
WAYPOINT = 'waypoint'


def wrap_angle(angle):
    """Return angle in the range [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def read_rows(rows):
    """Return waypoint rows as (x, y, yaw) tuples. Yaw is 0 when it is missing."""
    poses = []
    for row in rows:
        if len(row) not in (2, 3):
            raise ValueError('Waypoint must contain x, y and optional yaw')
        x = finite(row[0], 'waypoint.x')
        y = finite(row[1], 'waypoint.y')
        yaw = 0.0
        if len(row) == 3:
            yaw = finite(row[2], 'waypoint.yaw')
        poses.append((x, y, yaw))
    return poses


def requested_turn(poses, min_distance):
    """Return the heading change asked for when relative waypoints only turn on the spot.

    Returns None when any waypoint moves the robot at least min_distance.
    """
    if not poses:
        return None
    for x, y, _ in poses:
        if math.hypot(x, y) >= min_distance:
            return None
    return wrap_angle(poses[-1][2])


def arc_end(length, angle):
    """End pose of a circular arc that starts at the origin facing +x.

    A positive length drives forward, a negative one reverses. Either way the
    heading changes by angle (positive turns left).
    """
    if abs(angle) < 1e-6:
        return length, 0.0, 0.0
    radius = length / angle
    x = radius * math.sin(angle)
    y = radius * (1 - math.cos(angle))
    return x, y, angle


def turn_arcs(turn, length, max_angle):
    """Relative poses that turn the robot toward turn: a forward arc, then a reverse arc.

    The reverse arc is the first leg of a three-point turn, for when there is no room ahead.
    """
    angle = clamp(turn, max_angle)
    forward = arc_end(length, angle)
    reverse = arc_end(-length, angle)
    return [(FORWARD_ARC, forward), (REVERSE_ARC, reverse)]


def _place(pose, frame, origin, place):
    """Return pose as a free map pose, or None. Relative poses start at origin."""
    x, y, yaw = pose
    if frame != 'base_link':
        return place(x, y, yaw, frame)
    # Relative waypoints belong to the pose where the image was taken.
    x, y = transform_xy((x, y), origin)
    return place(x, y, yaw + origin[2], 'odom')


def _in_reach(goal, robot, limits):
    distance = math.hypot(goal[0] - robot[0], goal[1] - robot[1])
    if distance < limits['min_subgoal_distance_m']:
        return False
    return distance <= limits['max_subgoal_distance_m']


def choose_subgoal(rows, frame, origin, robot, place, limits):
    """Pick the next goal from a model's waypoints.

    rows: the model's waypoints, in frame. With frame 'base_link' they are relative
    to origin, the odom pose where the image was taken.
    robot: the robot's map pose now.
    place(x, y, yaw, frame): returns the goal as a map pose, or None if the robot
    cannot stand there.
    limits: the manager's parameters.

    Returns (map pose, kind), kind being WAYPOINT, FORWARD_ARC or REVERSE_ARC,
    or (None, None) when nothing can be driven.
    """
    poses = read_rows(rows)
    for pose in reversed(poses):
        goal = _place(pose, frame, origin, place)
        if goal is not None and _in_reach(goal, robot, limits):
            return goal, WAYPOINT

    # Only relative answers can ask for a turn on the spot.
    if frame != 'base_link' or not limits['turn_arcs']:
        return None, None
    turn = requested_turn(poses, limits['min_subgoal_distance_m'])
    if turn is None or abs(turn) < limits['min_turn_rad']:
        return None, None
    arcs = turn_arcs(turn, limits['turn_arc_length_m'], limits['turn_arc_max_rad'])
    for kind, pose in arcs:
        goal = _place(pose, frame, origin, place)
        if goal is not None and _in_reach(goal, robot, limits):
            return goal, kind
    return None, None
