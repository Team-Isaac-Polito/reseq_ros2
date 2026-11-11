"""Run a course of traction tasks and measure each leg against Gazebo ground truth.

Usage: ros2 run reseq_traction_autonomy traction_eval [course.yaml] [--repeat N]
"""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GazeboNode
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from .client import call
from .core import compose, inverse, yaw_of


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def rounded(values, digits=3):
    result = []
    for value in values:
        result.append(round(value, digits))
    return result


def find_pose_topic(gazebo, timeout_s=5.0):
    """Return Gazebo's dynamic pose topic. Wait a moment for Gazebo to list it."""
    deadline = time.monotonic() + timeout_s
    topics = []
    while not topics and time.monotonic() < deadline:
        for topic in gazebo.topic_list():
            if topic.endswith('/dynamic_pose/info'):
                topics.append(topic)
        time.sleep(0.2)
    if len(topics) != 1:
        raise RuntimeError(f'Expected one Gazebo world, found pose topics {topics}')
    return topics[0]


class Observer(Node):
    """Tracks base_link in Gazebo's world frame and in the SLAM map frame."""

    def __init__(self, model):
        super().__init__(
            'traction_eval', parameter_overrides=[Parameter('use_sim_time', value=True)]
        )
        self.model = model
        self.world_pose = None
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        # The ROS bridge drops entity names from Pose_V, so read Gazebo directly.
        self.gazebo = GazeboNode()
        self.pose_topic = find_pose_topic(self.gazebo)
        self.gazebo.subscribe(Pose_V, self.pose_topic, self._ground_truth_cb)

    def close(self):
        # Release the Gazebo node while Python is still running; destroying it during
        # interpreter shutdown aborts the process.
        self.gazebo.unsubscribe(self.pose_topic)
        self.gazebo = None
        self.destroy_node()

    def _ground_truth_cb(self, message):
        poses = {}
        for pose in message.pose:
            poses[pose.name] = (pose.position.x, pose.position.y, yaw_of(pose.orientation))
        if self.model not in poses:
            return
        pose = poses[self.model]
        if 'base_link' in poses:
            # Gazebo reports link poses relative to their model.
            pose = compose(pose, poses['base_link'])
        self.world_pose = pose

    def slam_pose(self):
        transform = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        t = transform.transform
        return (t.translation.x, t.translation.y, yaw_of(t.rotation))


def wait_for_poses(observer, timeout_s=30.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            slam = observer.slam_pose()
        except TransformException:
            slam = None
        if slam is not None and observer.world_pose is not None:
            return slam, observer.world_pose
        time.sleep(0.2)
    raise RuntimeError('No Gazebo pose for the robot model or no SLAM map pose')


def start_task(leg, tolerance):
    timeout_s = leg.get('timeout_s', 60.0)
    if leg.get('kind', 'metric') == 'metric':
        arguments = {
            'x': leg['x'],
            'y': leg['y'],
            'yaw': leg.get('yaw', 0.0),
            'frame': 'map',
            'timeout_s': timeout_s,
        }
        if 'zone_qr' in leg:
            arguments['zone_qr'] = leg['zone_qr']
        return call('navigate_to_pose', arguments)
    arguments = {
        'instruction': leg['instruction'],
        'model': leg.get('model', 'foresight'),
        'timeout_s': timeout_s,
        'goal_x': leg['x'],
        'goal_y': leg['y'],
        'goal_frame': 'map',
        'goal_tolerance': tolerance,
    }
    return call('start_visual_navigation', arguments)


def run_leg(observer, leg, world_to_map, tolerance):
    """Run one leg and measure it. Returns a result dict."""

    def true_pose():
        return compose(world_to_map, observer.world_pose)

    goal = (leg['x'], leg['y'])
    result = {'leg': leg['name'], 'kind': leg.get('kind', 'metric'), 'goal': list(goal)}
    try:
        task = start_task(leg, tolerance)
    except RuntimeError as exc:
        result.update(status='rejected', reason=str(exc), reached=False)
        return result

    # Follow the robot until the task ends.
    path_m = 0.0
    previous = true_pose()
    started = time.monotonic()
    trajectory = []
    while True:
        view = call('get_task', {'task_id': task['task_id']})
        if view['status'] != 'running':
            break
        time.sleep(0.2)
        current = true_pose()
        path_m += distance(current, previous)
        previous = current
        elapsed = round(time.monotonic() - started, 2)
        trajectory.append([elapsed] + rounded(current))

    # Let the robot come to rest before measuring where it ended.
    time.sleep(1.0)
    final = true_pose()
    slam = observer.slam_pose()
    true_error = distance(final, goal)
    result.update(
        status=view['status'],
        reason=view['reason'],
        task_id=task['task_id'],
        duration_s=view['elapsed_s'],
        path_m=round(path_m, 3),
        true_error_m=round(true_error, 3),
        slam_error_m=round(distance(slam, goal), 3),
        slam_drift_m=round(distance(slam, final), 3),
        final_true_pose=rounded(final),
        reached=true_error <= tolerance,
        trajectory=trajectory,
    )
    return result


def print_table(results):
    print(f'{"leg":16} {"status":10} {"true err":>9} {"slam err":>9} {"drift":>6}'
          f' {"path":>6} {"time":>6}  reached')
    reached_count = 0
    for r in results:
        if r['status'] == 'rejected':
            print(f'{r["leg"]:16} rejected   {r["reason"]}')
            continue
        reached = 'no'
        if r['reached']:
            reached = 'yes'
            reached_count += 1
        print(f'{r["leg"]:16} {r["status"]:10}'
              f' {r["true_error_m"]:8.2f}m {r["slam_error_m"]:8.2f}m {r["slam_drift_m"]:5.2f}m'
              f' {r["path_m"]:5.2f}m {r["duration_s"]:5.1f}s  {reached}')
    print(f'{reached_count}/{len(results)} legs reached within tolerance (Gazebo ground truth)')


def load_legs(course, only, repeat):
    """Return the legs to run: warm-up legs first, then the course `repeat` times."""
    legs = course['legs']
    if only:
        wanted = only.split(',')
        selected = []
        for leg in legs:
            if leg['name'] in wanted:
                selected.append(leg)
        legs = selected
    if repeat > 1:
        # Number each round: down_range_1, return_1, down_range_2, ...
        rounds = []
        for i in range(1, repeat + 1):
            for leg in legs:
                rounds.append(dict(leg, name=f'{leg["name"]}_{i}'))
        legs = rounds
    # Warm-up legs run once first, for example to let SLAM map the lane.
    return course.get('warmup', []) + legs


def parse_args():
    share = Path(get_package_share_directory('reseq_traction_autonomy'))
    default_course = share / 'config' / 'simple_course.yaml'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('course', nargs='?', default=str(default_course))
    parser.add_argument('--legs', help='comma-separated leg names to run')
    parser.add_argument('--repeat', type=int, help='rounds to run (default: from the course)')
    parser.add_argument('--model', default='reseq', help='Gazebo model name of the robot')
    parser.add_argument('--output', default='~/.local/state/reseq/traction/eval')
    return parser.parse_args()


def save_results(output, course_path, results):
    folder = Path(output).expanduser()
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f'{datetime.now():%Y%m%d-%H%M%S}.json'
    path.write_text(json.dumps({'course': course_path, 'results': results}, indent=2))
    print(f'results: {path}')


def main():
    args = parse_args()
    course = yaml.safe_load(Path(args.course).read_text())
    tolerance = course.get('tolerance', 0.25)
    repeat = args.repeat or course.get('repeat', 1)
    legs = load_legs(course, args.legs, repeat)

    rclpy.init()
    observer = Observer(args.model)
    executor = SingleThreadedExecutor()
    executor.add_node(observer)
    threading.Thread(target=executor.spin, daemon=True).start()
    results = []
    try:
        slam, world = wait_for_poses(observer)
        # SLAM and Gazebo agree at the start; later differences are SLAM drift.
        world_to_map = compose(slam, inverse(world))
        for leg in legs:
            print(f'running {leg["name"]} -> ({leg["x"]}, {leg["y"]})', flush=True)
            results.append(run_leg(observer, leg, world_to_map, tolerance))
    except KeyboardInterrupt:
        call('stop')
        print('stopped')
    finally:
        executor.shutdown()
        observer.close()
        rclpy.try_shutdown()

    print_table(results)
    save_results(args.output, args.course, results)


if __name__ == '__main__':
    main()
