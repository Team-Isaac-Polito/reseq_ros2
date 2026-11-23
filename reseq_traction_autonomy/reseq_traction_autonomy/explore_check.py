"""traction_explore_check: run frontier exploration on a world file, without ROS.

A repeatable check for exploration settings. The robot starts at the spawn point
and knows only what its lidar sees from there. It then picks a frontier, jumps to
the chosen pose, looks again, and repeats until nothing is left to explore. It
does not model driving: the path between poses is assumed to be free.

It prints each step and after which step the front camera first saw a target
point, such as the victim.

Usage:
  traction_explore_check worlds/rescue_maze.world --target 5.2 3.0
"""

import argparse
import math

from .core import Grid, padded_box
from .explore import UNKNOWN, choose_frontier
from .parameters import DEFAULTS
from .replay import straight_footprint, world_grid

LIDAR_RANGE_M = 6.0
RAY_COUNT = 360
MAX_STEPS = 40
# The arm camera: about 70 degrees wide, and a lying person is easy to spot up to 3 m.
CAMERA_RANGE_M = 3.0
CAMERA_HALF_FOV_RAD = 0.6
# The target is an obstacle itself; rays may stop this close to its centre.
TARGET_RADIUS_M = 0.45
# A goal with less free space than this around the head counts as tight.
TIGHT_MARGIN_M = 0.15


def blank_copy(grid):
    """A grid of the same size and place, with every cell unknown."""
    data = [UNKNOWN] * len(grid.data)
    return Grid(grid.width, grid.height, grid.resolution, grid.origin, data, grid.unknown_gap_m)


def look(known, truth, pose):
    """Copy into known what a 360 degree lidar at pose sees of truth. Rays stop at walls."""
    step = truth.resolution / 2
    for i in range(RAY_COUNT):
        angle = 2 * math.pi * i / RAY_COUNT
        distance = 0.0
        while distance <= LIDAR_RANGE_M:
            x = pose[0] + distance * math.cos(angle)
            y = pose[1] + distance * math.sin(angle)
            cell = truth.cell(x, y)
            if cell is None:
                break
            col, row = cell
            value = truth.value(col, row)
            index = row * truth.width + col
            if value == UNKNOWN:
                break
            known.data[index] = value
            if value != 0:
                break
            distance += step


def in_camera_view(truth, pose, target):
    """True if the front camera at pose would see target: close enough, inside the
    field of view and with no wall in between."""
    dx = target[0] - pose[0]
    dy = target[1] - pose[1]
    distance = math.hypot(dx, dy)
    if distance > CAMERA_RANGE_M:
        return False
    bearing = math.atan2(dy, dx) - pose[2]
    bearing = math.atan2(math.sin(bearing), math.cos(bearing))
    if abs(bearing) > CAMERA_HALF_FOV_RAD:
        return False
    # Walk toward the target; the target itself is an obstacle, so stop short of it.
    step = truth.resolution / 2
    travelled = 0.0
    while travelled < distance - TARGET_RADIUS_M:
        x = pose[0] + dx / distance * travelled
        y = pose[1] + dy / distance * travelled
        cell = truth.cell(x, y)
        if cell is not None and truth.value(cell[0], cell[1]) != 0:
            return False
        travelled += step
    return True


def free_share(known, truth):
    """Share of the truly free cells that are known."""
    free = 0
    found = 0
    for index, value in enumerate(truth.data):
        if value == 0:
            free += 1
            if known.data[index] == 0:
                found += 1
    return found / free


def explore(truth, cfg, target=None, start=(0.0, 0.0, 0.0)):
    """Run exploration and return a list of steps, one dict per chosen frontier."""
    # Goals are checked with the head, as on the robot.
    head = straight_footprint(dict(cfg, num_modules=1))
    roomy = padded_box(head, cfg['frontier_clearance_m'])
    # A goal is tight when the head has less than this free space around it.
    tight_check = padded_box(head, TIGHT_MARGIN_M)
    known = blank_copy(truth)

    def place(x, y, yaw):
        if not known.pose_free((x, y, yaw), head):
            return None
        if not known.pose_free((x, y, yaw), roomy):
            return None
        return x, y, yaw

    robot = start
    look(known, truth, robot)
    tried = []
    travelled = 0.0
    steps = []
    target_seen_at = None
    if target is not None and in_camera_view(truth, robot, target):
        target_seen_at = 0
    for number in range(1, MAX_STEPS + 1):
        goal, frontier = choose_frontier(known, robot, place, tried, cfg)
        if goal is None:
            break
        tried.append(frontier)
        travelled += math.hypot(goal[0] - robot[0], goal[1] - robot[1])
        robot = goal
        look(known, truth, robot)
        if target is not None and target_seen_at is None and in_camera_view(truth, robot, target):
            target_seen_at = number
        steps.append({
            'step': number,
            'goal': (round(goal[0], 2), round(goal[1], 2)),
            'travelled_m': round(travelled, 1),
            'explored': round(free_share(known, truth), 3),
            'target_seen': target_seen_at is not None,
            'tight': not truth.pose_free(goal, tight_check),
        })
    return steps, target_seen_at


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('world', help='the .world file')
    parser.add_argument('--target', nargs=2, type=float, metavar=('X', 'Y'),
                        help='a point to report first sight of, such as the victim')
    args = parser.parse_args()

    truth = world_grid(args.world)
    target = None
    if args.target:
        target = tuple(args.target)
    steps, target_seen_at = explore(truth, DEFAULTS, target)
    for step in steps:
        print(f'step {step["step"]:2d}  goal {step["goal"]}  travelled {step["travelled_m"]:5.1f} m'
              f'  explored {100 * step["explored"]:5.1f}%  target seen {step["target_seen"]}'
              f'  tight {step["tight"]}')
    if target is not None:
        print(f'target first seen after step {target_seen_at}')


if __name__ == '__main__':
    main()
