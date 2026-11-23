"""traction_replay: replay a visual task's recorded predictions through subgoal selection.

It needs no ROS, no simulation and no model server, and the same task record always
gives the same numbers. So it is the baseline check for changes to subgoal selection:
it counts how many model answers become a goal the robot can drive, once with turn
arcs and once without.

The map is built from the world file's collision boxes and cylinders, as if SLAM had
mapped everything perfectly, and the robot is assumed straight. It does not drive the
robot, so it does not replace a live run.

Usage:
  traction_replay ~/.local/state/reseq/traction/TASK_ID worlds/rescue_maze.world
"""

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from .core import Grid, compose, convex_hull
from .parameters import DEFAULTS
from .subgoals import choose_subgoal

RESOLUTION = 0.05
# Distance between module centres when the body is straight (from TF in Gazebo).
MODULE_SPACING = 0.398
# Grid steps to the four cells that share a side with a cell.
NEIGHBOURS = ((1, 0), (-1, 0), (0, 1), (0, -1))


# ---- Map from the world file ----


def read_pose(element):
    """Return (x, y, yaw) from an SDF <pose> child, or the origin if there is none."""
    text = element.findtext('pose')
    if not text:
        return 0.0, 0.0, 0.0
    values = [float(v) for v in text.split()]
    return values[0], values[1], values[5]


def world_shapes(world_path):
    """Return the world's obstacles as ('box', pose, (sx, sy)) or ('circle', pose, radius)."""
    root = ET.parse(world_path).getroot()
    shapes = []
    for model in root.iter('model'):
        model_pose = read_pose(model)
        for collision in model.iter('collision'):
            pose = compose(model_pose, read_pose(collision))
            box = collision.find('geometry/box/size')
            circle = collision.find('geometry/cylinder/radius')
            if box is not None:
                size = [float(v) for v in box.text.split()]
                shapes.append(('box', pose, (size[0], size[1])))
            elif circle is not None:
                shapes.append(('circle', pose, float(circle.text)))
    return shapes


def inside(point, shape):
    """True if a map point lies inside an obstacle shape."""
    kind, pose, size = shape
    # The point in the shape's own frame.
    dx = point[0] - pose[0]
    dy = point[1] - pose[1]
    along = math.cos(pose[2]) * dx + math.sin(pose[2]) * dy
    across = -math.sin(pose[2]) * dx + math.cos(pose[2]) * dy
    if kind == 'circle':
        return math.hypot(along, across) <= size
    return abs(along) <= size[0] / 2 and abs(across) <= size[1] / 2


def mark_unreachable_unknown(grid, start):
    """Keep free only the cells reachable from start; the rest become unknown (-1).

    SLAM never sees past the outer walls, so neither should the replay map.
    """
    reachable = [False] * len(grid.data)
    col, row = grid.cell(start[0], start[1])
    queue = [(col, row)]
    reachable[row * grid.width + col] = True
    while queue:
        col, row = queue.pop()
        for step_col, step_row in NEIGHBOURS:
            next_col = col + step_col
            next_row = row + step_row
            if not (0 <= next_col < grid.width and 0 <= next_row < grid.height):
                continue
            index = next_row * grid.width + next_col
            if reachable[index] or grid.data[index] != 0:
                continue
            reachable[index] = True
            queue.append((next_col, next_row))
    for index, value in enumerate(grid.data):
        if value == 0 and not reachable[index]:
            grid.data[index] = -1


def world_grid(world_path, start=(0.0, 0.0)):
    """Build an occupancy grid of the world: 100 in obstacles, 0 in free space reachable
    from start (the robot's spawn point), and -1 (unknown) elsewhere."""
    shapes = world_shapes(world_path)
    # The grid's bounds: every shape fits in a circle of this reach around its centre.
    min_x = min_y = math.inf
    max_x = max_y = -math.inf
    for kind, pose, size in shapes:
        if kind == 'box':
            reach = math.hypot(size[0], size[1]) / 2
        else:
            reach = size
        min_x = min(min_x, pose[0] - reach)
        max_x = max(max_x, pose[0] + reach)
        min_y = min(min_y, pose[1] - reach)
        max_y = max(max_y, pose[1] + reach)
    width = math.ceil((max_x - min_x) / RESOLUTION)
    height = math.ceil((max_y - min_y) / RESOLUTION)
    data = [0] * (width * height)
    for row in range(height):
        for col in range(width):
            centre = (min_x + (col + 0.5) * RESOLUTION, min_y + (row + 0.5) * RESOLUTION)
            for shape in shapes:
                if inside(centre, shape):
                    data[row * width + col] = 100
                    break
    grid = Grid(width, height, RESOLUTION, (min_x, min_y, 0.0), data)
    mark_unreachable_unknown(grid, start)
    return grid


def straight_footprint(cfg):
    """The footprint hull of the straight robot in base_link, padded like the live one."""
    a = cfg['module_half_length']
    b = cfg['module_half_width']
    points = []
    for index in range(int(cfg['num_modules'])):
        centre = -index * MODULE_SPACING
        front = a
        if index == 0:
            front = a + cfg['camera_standoff_m']
        points += [(centre - a, -b), (centre + front, -b), (centre + front, b), (centre - a, b)]
    return convex_hull(points)


# ---- Replay ----


def read_events(path):
    path = Path(path).expanduser()
    if path.is_dir():
        path = path / 'events.jsonl'
    events = []
    for line in path.read_text().splitlines():
        events.append(json.loads(line))
    return events


def replay(events, grid, footprint, cfg):
    """Return a Counter of what each prediction became: a subgoal kind, 'none', 'stop',
    or 'stop_at_wall' for a stop the manager ignores because it faces a wall.

    Poses come from the record: the map pose logged just before the prediction's image
    was taken is where the waypoints start. In the replay map, odom and map are the same.
    """
    def place(x, y, yaw, frame):
        if grid.pose_free((x, y, yaw), footprint):
            return x, y, yaw
        return None

    counts = Counter()
    robot = None
    origin = None
    for event in events:
        kind = event['event']
        data = event['data']
        if kind == 'state' and data.get('pose_frame') == 'map':
            pose = data['pose']
            robot = (pose['x'], pose['y'], pose['yaw'])
        elif kind == 'observation':
            origin = robot
        elif kind == 'prediction':
            if origin is None or robot is None:
                continue
            if data['stop']:
                # Same check as the manager: a stop facing a wall is ignored.
                front = cfg['module_half_length']
                if grid.blocked_ahead(robot, front, cfg['stop_wall_distance_m']):
                    counts['stop_at_wall'] += 1
                else:
                    counts['stop'] += 1
                continue
            goal, chosen = choose_subgoal(
                data['waypoints'], data['frame'], origin, robot, place, cfg
            )
            if goal is None:
                counts['none'] += 1
            else:
                counts[chosen] += 1
    return counts


def print_counts(label, counts):
    total = sum(counts.values())
    print(f'{label}: {total} predictions')
    for kind in ('waypoint', 'forward_arc', 'reverse_arc', 'none', 'stop', 'stop_at_wall'):
        share = 0.0
        if total:
            share = 100.0 * counts[kind] / total
        print(f'  {kind:<12} {counts[kind]:>5}  {share:5.1f}%')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('task', help='task record folder or its events.jsonl')
    parser.add_argument('world', help='the .world file the task ran in')
    args = parser.parse_args()

    events = read_events(args.task)
    grid = world_grid(args.world)
    footprint = straight_footprint(DEFAULTS)
    without = dict(DEFAULTS, turn_arcs=False)
    print_counts('without turn arcs', replay(events, grid, footprint, without))
    print_counts('with turn arcs', replay(events, grid, footprint, DEFAULTS))


if __name__ == '__main__':
    main()
