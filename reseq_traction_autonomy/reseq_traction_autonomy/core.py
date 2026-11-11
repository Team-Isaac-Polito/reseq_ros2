"""Task lifetime and planar geometry, independent of ROS.

Poses are (x, y, yaw) tuples in metres and radians.
"""

from dataclasses import dataclass, field
import math
import time
import uuid

TERMINAL = {'succeeded', 'cancelled', 'timed_out', 'failed'}

# Occupancy grid values: -1 is unknown, 0 is free, 100 is occupied.
# Cells at or above this value count as obstacles.
OCCUPIED = 65


def finite(value, name):
    """Return value as a float, or raise if it is not a finite number."""
    # bool is a subclass of int, so reject it explicitly.
    if isinstance(value, bool):
        raise ValueError(f'{name} must be a finite number')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return value


def clamp(value, limit):
    """Limit value to the range [-limit, limit]."""
    return max(-limit, min(limit, value))


def yaw_of(q):
    """Return the yaw angle of a quaternion."""
    sin_yaw = 2 * (q.w * q.z + q.x * q.y)
    cos_yaw = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(sin_yaw, cos_yaw)


def transform_xy(point, pose):
    """Move a point from the frame of pose into the frame pose is given in."""
    x, y, yaw = pose
    px, py = point
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    new_x = x + cos_yaw * px - sin_yaw * py
    new_y = y + sin_yaw * px + cos_yaw * py
    return new_x, new_y


def compose(a, b):
    """Return pose b, given relative to pose a, in the frame of a."""
    x, y = transform_xy((b[0], b[1]), a)
    yaw = a[2] + b[2]
    return x, y, yaw


def inverse(pose):
    """Return the pose that undoes pose."""
    x, y, yaw = pose
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    new_x = -cos_yaw * x - sin_yaw * y
    new_y = sin_yaw * x - cos_yaw * y
    return new_x, new_y, -yaw


def _cross(o, a, b):
    """Positive when o -> a -> b turns left."""
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _half_hull(points):
    """Build one side of the hull, dropping points that turn the wrong way."""
    hull = []
    for point in points:
        while len(hull) >= 2 and _cross(hull[-2], hull[-1], point) <= 0:
            hull.pop()
        hull.append(point)
    return hull


def convex_hull(points):
    """Return the convex hull of 2D points, counter-clockwise."""
    points = sorted(set(points))
    if len(points) <= 2:
        return points
    lower = _half_hull(points)
    upper = _half_hull(reversed(points))
    # The last point of each half is the first point of the other.
    return lower[:-1] + upper[:-1]


def point_in_polygon(point, polygon):
    """Ray casting: count how many polygon edges a ray to the right crosses."""
    x, y = point
    inside = False
    for i in range(len(polygon)):
        a = polygon[i]
        b = polygon[(i + 1) % len(polygon)]
        crosses_row = (a[1] > y) != (b[1] > y)
        if not crosses_row:
            continue
        edge_x = a[0] + (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1])
        if x < edge_x:
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
        """Return the (col, row) of a map point, or None outside the grid."""
        origin_x, origin_y, yaw = self.origin
        dx = x - origin_x
        dy = y - origin_y
        # Rotate into the grid's own axes.
        along = math.cos(yaw) * dx + math.sin(yaw) * dy
        across = -math.sin(yaw) * dx + math.cos(yaw) * dy
        col = math.floor(along / self.resolution)
        row = math.floor(across / self.resolution)
        if col < 0 or col >= self.width:
            return None
        if row < 0 or row >= self.height:
            return None
        return col, row

    def value(self, col, row):
        return self.data[row * self.width + col]

    def _near_free_cell(self, col, row):
        """True if a free cell lies within unknown_gap_m of this cell."""
        reach = round(self.unknown_gap_m / self.resolution)
        first_col = max(col - reach, 0)
        last_col = min(col + reach, self.width - 1)
        first_row = max(row - reach, 0)
        last_row = min(row + reach, self.height - 1)
        for c in range(first_col, last_col + 1):
            for r in range(first_row, last_row + 1):
                value = self.value(c, r)
                if 0 <= value < OCCUPIED:
                    return True
        return False

    def traversable(self, x, y):
        """True if the robot may stand on this map point."""
        cell = self.cell(x, y)
        if cell is None:
            return False
        col, row = cell
        value = self.value(col, row)
        if value >= OCCUPIED:
            return False
        if value >= 0:
            return True
        return self._near_free_cell(col, row)

    def pose_free(self, pose, footprint):
        """True if the whole footprint, placed at pose, is on traversable cells."""
        polygon = []
        for corner in footprint:
            polygon.append(transform_xy(corner, pose))
        for x, y in polygon:
            if not self.traversable(x, y):
                return False

        # Corners alone miss obstacles under the body, so also sample the inside.
        xs = [p[0] for p in polygon]
        ys = [p[1] for p in polygon]
        step = self.resolution / 2
        x_steps = math.ceil((max(xs) - min(xs)) / step)
        y_steps = math.ceil((max(ys) - min(ys)) / step)
        for ix in range(x_steps + 1):
            for iy in range(y_steps + 1):
                x = min(xs) + ix * step
                y = min(ys) + iy * step
                if not point_in_polygon((x, y), polygon):
                    continue
                if not self.traversable(x, y):
                    return False
        return self.traversable(pose[0], pose[1])


def new_task_id():
    return uuid.uuid4().hex


@dataclass
class Task:
    kind: str
    arguments: dict
    timeout_s: float
    created: float
    id: str = field(default_factory=new_task_id)
    status: str = 'running'
    reason: str = ''
    subgoal: object = None
    progress: dict = field(default_factory=dict)
    finished: float | None = None

    def view(self, now):
        """Return the task as a JSON-friendly dict."""
        end = now
        if self.finished is not None:
            end = self.finished
        return {
            'task_id': self.id,
            'kind': self.kind,
            'status': self.status,
            'reason': self.reason,
            'elapsed_s': round(end - self.created, 3),
            'timeout_s': self.timeout_s,
            'subgoal': self.subgoal,
            'progress': self.progress,
            'arguments': self.arguments,
        }


class Tasks:
    """All tasks of this run. At most one is active at a time."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.items = {}
        self.active = None
        # Changes whenever a task starts or ends. Callbacks compare it to
        # ignore replies that belong to an older task.
        self.generation = 0

    def start(self, kind, arguments, timeout_s):
        timeout_s = finite(timeout_s, 'timeout_s')
        if timeout_s < 1 or timeout_s > 600:
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
        if task is None:
            return None
        task.status = status
        task.reason = reason
        task.finished = self.clock()
        self.active = None
        self.generation += 1
        return task

    def current(self, task_id, generation):
        """True if this task is still the active one and has not restarted."""
        if self.active is None:
            return False
        if self.active.id != task_id:
            return False
        return self.generation == generation

    def expired(self):
        if self.active is None:
            return False
        age = self.clock() - self.active.created
        return age > self.active.timeout_s
