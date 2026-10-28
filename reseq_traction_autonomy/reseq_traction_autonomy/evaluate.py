"""Run a course of traction tasks and measure each leg against Gazebo ground truth."""

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
        topics = []
        deadline = time.monotonic() + 5.0
        while not topics and time.monotonic() < deadline:
            topics = [t for t in self.gazebo.topic_list() if t.endswith('/dynamic_pose/info')]
            time.sleep(0.2)
        if len(topics) != 1:
            raise RuntimeError(f'Expected one Gazebo world, found pose topics {topics}')
        self.pose_topic = topics[0]
        self.gazebo.subscribe(Pose_V, self.pose_topic, self._ground_truth_cb)

    def close(self):
        # Stop Gazebo callbacks before Python exits; otherwise its thread aborts the process.
        self.gazebo.unsubscribe(self.pose_topic)
        self.destroy_node()

    def _ground_truth_cb(self, message):
        poses = {}
        for pose in message.pose:
            q = pose.orientation
            poses[pose.name] = (pose.position.x, pose.position.y, yaw_of(q))
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
    def true_pose():
        return compose(world_to_map, observer.world_pose)

    result = {'leg': leg['name'], 'kind': leg.get('kind', 'metric'), 'goal': [leg['x'], leg['y']]}
    start = true_pose()
    try:
        task = start_task(leg, tolerance)
    except RuntimeError as exc:
        result.update(status='rejected', reason=str(exc), reached=False)
        return result
    path_m = 0.0
    previous = start
    started = time.monotonic()
    trajectory = []
    while True:
        view = call('get_task', {'task_id': task['task_id']})
        if view['status'] != 'running':
            break
        time.sleep(0.2)
        current = true_pose()
        path_m += math.hypot(current[0] - previous[0], current[1] - previous[1])
        previous = current
        trajectory.append([round(time.monotonic() - started, 2)] + [round(v, 3) for v in current])
    # Let the robot come to rest before measuring where it ended.
    time.sleep(1.0)
    final = true_pose()
    slam = observer.slam_pose()
    true_error = math.hypot(final[0] - leg['x'], final[1] - leg['y'])
    result.update(
        status=view['status'],
        reason=view['reason'],
        task_id=task['task_id'],
        duration_s=view['elapsed_s'],
        path_m=round(path_m, 3),
        true_error_m=round(true_error, 3),
        slam_error_m=round(math.hypot(slam[0] - leg['x'], slam[1] - leg['y']), 3),
        slam_drift_m=round(math.hypot(slam[0] - final[0], slam[1] - final[1]), 3),
        final_true_pose=[round(v, 3) for v in final],
        reached=true_error <= tolerance,
        trajectory=trajectory,
    )
    return result


def print_table(results):
    header = f'{"leg":16} {"status":10} {"true err":>9} {"slam err":>9} {"drift":>6}'
    print(header + f' {"path":>6} {"time":>6}  reached')
    for r in results:
        if r['status'] == 'rejected':
            print(f'{r["leg"]:16} rejected   {r["reason"]}')
            continue
        print(
            f'{r["leg"]:16} {r["status"]:10} {r["true_error_m"]:8.2f}m {r["slam_error_m"]:8.2f}m'
            f' {r["slam_drift_m"]:5.2f}m {r["path_m"]:5.2f}m {r["duration_s"]:5.1f}s'
            f'  {"yes" if r["reached"] else "no"}'
        )
    reached = sum(1 for r in results if r['reached'])
    print(f'{reached}/{len(results)} legs reached within tolerance (Gazebo ground truth)')


def main():
    default_course = Path(get_package_share_directory('reseq_traction_autonomy'))
    default_course = default_course / 'config' / 'simple_course.yaml'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('course', nargs='?', default=str(default_course))
    parser.add_argument('--legs', help='comma-separated leg names to run')
    parser.add_argument('--model', default='reseq', help='Gazebo model name of the robot')
    parser.add_argument('--output', default='~/.local/state/reseq/traction/eval')
    args = parser.parse_args()

    course = yaml.safe_load(Path(args.course).read_text())
    tolerance = course.get('tolerance', 0.25)
    legs = course['legs']
    if args.legs:
        wanted = args.legs.split(',')
        legs = [leg for leg in legs if leg['name'] in wanted]

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
    output = Path(args.output).expanduser()
    output.mkdir(parents=True, exist_ok=True)
    path = output / f'{datetime.now():%Y%m%d-%H%M%S}.json'
    path.write_text(json.dumps({'course': args.course, 'results': results}, indent=2))
    print(f'results: {path}')


if __name__ == '__main__':
    main()
