"""Task lifetime and planar geometry, independent of ROS."""

from dataclasses import dataclass, field
import math
import time
import uuid

TERMINAL = {'succeeded', 'cancelled', 'timed_out', 'failed'}


def finite(value, name):
    if isinstance(value, bool):
        raise ValueError(f'{name} must be a finite number')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return value


def clamp(value, limit):
    return max(-limit, min(limit, value))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def transform_xy(point, pose):
    x, y, yaw = pose
    px, py = point
    return (
        x + math.cos(yaw) * px - math.sin(yaw) * py,
        y + math.sin(yaw) * px + math.cos(yaw) * py,
    )


def compose(a, b):
    """Return pose b, given relative to pose a, in the frame of a."""
    x, y = transform_xy((b[0], b[1]), a)
    return (x, y, a[2] + b[2])


def inverse(pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (-c * x - s * y, s * x - c * y, -yaw)


def convex_hull(points):
    points = sorted(set(points))
    if len(points) <= 2:
        return points

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if (a[1] > y) != (b[1] > y):
            intercept = (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]
            if x < intercept:
                inside = not inside
    return inside


@dataclass
class Grid:
    width: int
    height: int
    resolution: float
    origin: tuple
    data: object
    # Unknown cells this close to a known free cell count as free. A lidar scan taken
    # from one spot leaves such gaps between its rays.
    unknown_gap_m: float = 0.0

    def cell(self, x, y):
        ox, oy, yaw = self.origin
        dx, dy = x - ox, y - oy
        col = math.floor((math.cos(yaw) * dx + math.sin(yaw) * dy) / self.resolution)
        row = math.floor((-math.sin(yaw) * dx + math.cos(yaw) * dy) / self.resolution)
        if not (0 <= col < self.width and 0 <= row < self.height):
            return None
        return col, row

    def value(self, col, row):
        return self.data[row * self.width + col]

    def traversable(self, x, y):
        cell = self.cell(x, y)
        if cell is None:
            return False
        col, row = cell
        value = self.value(col, row)
        if value >= 65:
            return False
        if value >= 0:
            return True
        reach = round(self.unknown_gap_m / self.resolution)
        for c in range(max(col - reach, 0), min(col + reach + 1, self.width)):
            for r in range(max(row - reach, 0), min(row + reach + 1, self.height)):
                if 0 <= self.value(c, r) < 65:
                    return True
        return False

    def pose_free(self, pose, footprint):
        polygon = [transform_xy(p, pose) for p in footprint]
        if not all(self.traversable(x, y) for x, y in polygon):
            return False
        # Sample the filled footprint; vertices alone miss obstacles beneath the body.
        step = self.resolution / 2
        min_x, max_x = min(p[0] for p in polygon), max(p[0] for p in polygon)
        min_y, max_y = min(p[1] for p in polygon), max(p[1] for p in polygon)
        for ix in range(math.ceil((max_x - min_x) / step) + 1):
            for iy in range(math.ceil((max_y - min_y) / step) + 1):
                x, y = min_x + ix * step, min_y + iy * step
                if point_in_polygon((x, y), polygon) and not self.traversable(x, y):
                    return False
        return self.traversable(pose[0], pose[1])


@dataclass
class Task:
    kind: str
    arguments: dict
    timeout_s: float
    created: float
    id: str = field(default_factory=lambda: uuid.uuid4().hex)
    status: str = 'running'
    reason: str = ''
    subgoal: object = None
    progress: dict = field(default_factory=dict)
    finished: float | None = None

    def view(self, now):
        return {
            'task_id': self.id,
            'kind': self.kind,
            'status': self.status,
            'reason': self.reason,
            'elapsed_s': round((self.finished or now) - self.created, 3),
            'timeout_s': self.timeout_s,
            'subgoal': self.subgoal,
            'progress': self.progress,
            'arguments': self.arguments,
        }


class Tasks:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.items = {}
        self.active = None
        self.generation = 0

    def start(self, kind, arguments, timeout_s):
        timeout_s = finite(timeout_s, 'timeout_s')
        if not 1 <= timeout_s <= 600:
            raise ValueError('timeout_s must be between 1 and 600')
        if self.active:
            raise ValueError('A task is running; cancel it before starting another')
        task = Task(kind, arguments, timeout_s, self.clock())
        self.items[task.id] = task
        self.active = task
        self.generation += 1
        return task

    def finish(self, status, reason=''):
        if status not in TERMINAL:
            raise ValueError('Invalid terminal status')
        task = self.active
        if task:
            task.status, task.reason, task.finished = status, reason, self.clock()
            self.active = None
            self.generation += 1
        return task

    def current(self, task_id, generation):
        return bool(self.active and self.active.id == task_id and self.generation == generation)

    def expired(self):
        return bool(self.active and self.clock() - self.active.created > self.active.timeout_s)
