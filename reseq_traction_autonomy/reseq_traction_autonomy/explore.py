"""Frontier exploration on the SLAM map, independent of ROS.

A frontier is where known free space meets unknown space, such as a doorway
into a room the robot has not seen yet. Driving to one frontier after another,
nearest and largest first, explores the whole map. It gives a visual task the
plan and the memory that the navigation model does not have.
"""

import math

from .core import OCCUPIED, transform_xy

# Grid steps to the cells that share a side, and to all eight around a cell.
SIDES = ((1, 0), (-1, 0), (0, 1), (0, -1))
AROUND = SIDES + ((1, 1), (1, -1), (-1, 1), (-1, -1))
UNKNOWN = -1


def _inside(grid, col, row):
    return 0 <= col < grid.width and 0 <= row < grid.height


def _is_free(value):
    return 0 <= value < OCCUPIED


def free_area(grid):
    """Return the mapped free area in square metres."""
    free_cells = 0
    for value in grid.data:
        if _is_free(value):
            free_cells += 1
    return free_cells * grid.resolution * grid.resolution


def frontier_cells(grid):
    """Return the free cells that share a side with an unknown cell."""
    cells = []
    for row in range(grid.height):
        for col in range(grid.width):
            if not _is_free(grid.value(col, row)):
                continue
            for step_col, step_row in SIDES:
                next_col = col + step_col
                next_row = row + step_row
                if not _inside(grid, next_col, next_row):
                    continue
                if grid.value(next_col, next_row) == UNKNOWN:
                    cells.append((col, row))
                    break
    return cells


def group_cells(cells):
    """Split cells into groups of cells that touch, including at corners."""
    remaining = set(cells)
    groups = []
    while remaining:
        start = remaining.pop()
        group = [start]
        queue = [start]
        while queue:
            col, row = queue.pop()
            for step_col, step_row in AROUND:
                neighbour = (col + step_col, row + step_row)
                if neighbour in remaining:
                    remaining.remove(neighbour)
                    group.append(neighbour)
                    queue.append(neighbour)
        groups.append(group)
    return groups


def cell_centre(grid, col, row):
    """Return the map point at the centre of a cell."""
    along = (col + 0.5) * grid.resolution
    across = (row + 0.5) * grid.resolution
    return transform_xy((along, across), grid.origin)


def find_frontiers(grid, min_size_m):
    """Return frontiers as (x, y, size in metres), ignoring ones shorter than min_size_m.

    The point is the frontier cell closest to the group's average, so it always
    lies on the frontier, also when the frontier is curved.
    """
    frontiers = []
    for group in group_cells(frontier_cells(grid)):
        size = len(group) * grid.resolution
        if size < min_size_m:
            continue
        points = []
        for col, row in group:
            points.append(cell_centre(grid, col, row))
        mean_x = sum(p[0] for p in points) / len(points)
        mean_y = sum(p[1] for p in points) / len(points)
        closest = points[0]
        closest_distance = math.inf
        for point in points:
            distance = math.hypot(point[0] - mean_x, point[1] - mean_y)
            if distance < closest_distance:
                closest = point
                closest_distance = distance
        frontiers.append((closest[0], closest[1], size))
    return frontiers


def viewing_poses(frontier, robot, standoffs):
    """Poses facing the frontier, on the line from it back toward the robot.

    The camera is at the front, so facing the frontier shows what lies beyond it.
    """
    dx = robot[0] - frontier[0]
    dy = robot[1] - frontier[1]
    distance = math.hypot(dx, dy)
    if distance < 1e-6:
        # Standing on it already: keep the current heading.
        dx = -math.cos(robot[2])
        dy = -math.sin(robot[2])
        distance = 1.0
    back_x = dx / distance
    back_y = dy / distance
    yaw = math.atan2(-back_y, -back_x)
    poses = []
    for standoff in standoffs:
        poses.append((frontier[0] + back_x * standoff, frontier[1] + back_y * standoff, yaw))
    return poses


def _already_tried(frontier, tried, radius):
    for x, y in tried:
        if math.hypot(frontier[0] - x, frontier[1] - y) < radius:
            return True
    return False


def choose_frontier(grid, robot, place, tried, cfg):
    """Pick the next frontier to look at.

    robot: the robot's map pose. place(x, y, yaw): the pose as a free map pose,
    or None. tried: (x, y) of frontiers chosen before; frontiers near them are skipped.
    Nearer and larger frontiers score better.

    Returns (goal pose, frontier point), or (None, None) when nothing is left to explore.
    """
    scored = []
    for frontier in find_frontiers(grid, cfg['frontier_min_size_m']):
        if _already_tried(frontier, tried, cfg['frontier_tried_radius_m']):
            continue
        distance = math.hypot(frontier[0] - robot[0], frontier[1] - robot[1])
        # Right around the robot the map is still filling in; that is not a place to visit.
        if distance < cfg['frontier_min_distance_m']:
            continue
        score = distance - cfg['frontier_size_weight'] * frontier[2]
        scored.append((score, frontier))
    scored.sort()
    for _, frontier in scored:
        for pose in viewing_poses(frontier, robot, cfg['frontier_standoffs_m']):
            goal = place(pose[0], pose[1], pose[2])
            if goal is not None:
                return goal, (frontier[0], frontier[1])
    return None, None
