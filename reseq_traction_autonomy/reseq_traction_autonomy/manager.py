"""Traction mission lifecycle, navigation goals and model requests."""

import json
import math
from pathlib import Path
import queue
import time
from importlib import import_module

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .core import finite, transform_xy
from .models import LightNav, foresight_waypoints
from .ros_interface import RosInterface, stamp_seconds


class Manager(RosInterface):
    def _start(self, kind, arguments, timeout_s):
        problems = self._readiness(visual=kind == 'visual')
        if problems:
            raise ValueError('Not ready: ' + ', '.join(problems))
        if self.count_publishers('/cmd_vel_nav') != 1:
            raise ValueError('Expected this gate to be the sole /cmd_vel_nav publisher')
        if self.count_publishers('/cmd_vel') != 1:
            raise ValueError('Expected exactly one command mux publisher')
        if 'autonomy_coordinator' in self.get_node_names():
            raise ValueError('The existing autonomy_coordinator must be excluded from this launch')
        if not self.enable_client.service_is_ready():
            raise ValueError('/autonomy/enable is unavailable')
        task = self.tasks.start(kind, arguments, timeout_s)
        try:
            self._open_record(task)
        except OSError:
            self.tasks.finish('failed', 'Could not create task record')
            raise
        self.last_candidate = Twist()
        self.candidate_time = 0.0
        self.last_prediction = None
        self.next_prediction = 0
        future = self.enable_client.call_async(SetBool.Request(data=True))
        task_id, generation = task.id, self.tasks.generation

        def enabled(done):
            if self.tasks.current(task_id, generation):
                try:
                    if done.result().success:
                        self.enabled_generation = generation
                    else:
                        self._finish('failed', 'Autonomy enable rejected')
                except Exception as exc:
                    self._finish('failed', str(exc))

        future.add_done_callback(enabled)
        self._event('task_started', task.view(time.monotonic()))
        return task

    def _open_record(self, task):
        directory = Path(self.cfg['record_directory']).expanduser() / task.id
        directory.mkdir(parents=True, exist_ok=False)
        self.record_dir = directory
        self.log_file = (directory / 'events.jsonl').open('a')
        self.recorded_images = 0

    def _event(self, kind, data):
        if not self.log_file:
            return
        record = {
            'event': kind,
            'wall_s': time.time(),
            'ros_s': self.get_clock().now().nanoseconds / 1e9,
            'data': data,
        }
        self.log_file.write(json.dumps(record, allow_nan=False) + '\n')
        self.log_file.flush()

    def _finish(self, status, reason):
        task = self.tasks.finish(status, reason)
        self.nav_epoch += 1
        self.nav_pending = False
        if self.model_future is not None:
            if self.model_context['model'] == 'foresight':
                self.foresight.remove_pending_request(self.model_future)
            self.model_future.cancel()
            self.model_future = None
            self.model_context = None
        # Publishing fails once rclpy has shut down; the mux then stops on stale commands.
        if rclpy.ok():
            self.output.publish(Twist())
            if self.nav_goal is not None:
                self.nav_goal.cancel_goal_async()
            # Return the mux to teleop so joystick input below the override deadband works.
            if (task or self.autonomy_enabled) and self.enable_client.service_is_ready():
                self.enable_client.call_async(SetBool.Request(data=False))
        self.nav_goal = None
        if task:
            self._event('task_finished', task.view(time.monotonic()))
        if self.log_file:
            self.log_file.close()
            self.log_file = None
        self.gate_reason = 'idle'

    def _validate_goal(self, x, y, yaw, frame):
        if self.grid is None or not self.footprint:
            raise ValueError('Map and robot footprint must be ready before setting a goal')
        if frame not in ('map', 'odom'):
            raise ValueError('frame must be map or odom')
        if frame != self.map_frame:
            frame_pose = self._pose(self.map_frame, source=frame)
            x, y = transform_xy((x, y), frame_pose)
            yaw += frame_pose[2]
        if not self.grid.pose_free((x, y, yaw), self.footprint):
            raise ValueError('Goal footprint intersects occupied, unknown or unmapped cells')
        return x, y, yaw

    def _send_goal(self, x, y, yaw, frame):
        task = self.tasks.active
        if task is None or self.nav_goal is not None or self.nav_pending:
            return
        x, y, yaw = self._validate_goal(x, y, yaw, frame)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        task.subgoal = {'x': x, 'y': y, 'yaw': yaw, 'frame': self.map_frame}
        self.nav_pending = True
        self.nav_epoch += 1
        task_id, generation, epoch = task.id, self.tasks.generation, self.nav_epoch
        future = self.nav.send_goal_async(
            goal, feedback_callback=lambda msg: self._feedback(msg, task_id, generation, epoch)
        )

        def accepted(done):
            handle = done.result()
            if not self.tasks.current(task_id, generation) or self.nav_epoch != epoch:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            self.nav_pending = False
            if not handle.accepted:
                self._finish('failed', 'Nav2 rejected the goal')
                return
            self.nav_goal = handle
            result = handle.get_result_async()
            result.add_done_callback(lambda f: self._nav_result(f, task_id, generation, epoch))

        future.add_done_callback(accepted)
        self._event('subgoal_sent', task.subgoal)

    def _feedback(self, message, task_id, generation, epoch):
        if self.tasks.current(task_id, generation) and self.nav_epoch == epoch:
            self.tasks.active.progress['distance_remaining_m'] = (
                message.feedback.distance_remaining
            )

    def _nav_result(self, future, task_id, generation, epoch):
        if not self.tasks.current(task_id, generation) or self.nav_epoch != epoch:
            return
        self.nav_goal = None
        self.nav_pending = False
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._finish('failed', f'Nav2 ended with status {status}')
        elif self.tasks.active.kind == 'metric':
            self._finish('succeeded', 'Nav2 reported success; position not independently measured')
        else:
            task = self.tasks.active
            task.progress['completed_subgoals'] = task.progress.get('completed_subgoals', 0) + 1
            task.subgoal = None
            self.next_prediction = 0
            self._event('subgoal_reached', task.progress)

    def _foresight_client(self):
        if self.foresight is None:
            from rosidl_runtime_py.utilities import get_service

            self.foresight_type = get_service('amrl_msgs/srv/ForesightPlannerSrv')
            self.foresight = self.create_client(self.foresight_type, self.cfg['foresight_service'])
        return self.foresight

    def _lightnav_predict(self, task_id, jpeg, instruction):
        if self.lightnav_task != task_id:
            if self.lightnav is not None:
                self.lightnav.close()
            self.lightnav = LightNav(self.cfg['lightnav_url'], self.cfg['model_timeout_s'])
            self.lightnav_task = task_id
        return self.lightnav.predict(jpeg, instruction)

    def _request_prediction(self):
        task = self.tasks.active
        observation = self._observation()
        self.model_context = {
            'id': task.id,
            'generation': self.tasks.generation,
            'created': time.monotonic(),
            'observation': observation,
            'model': task.arguments['model'],
        }
        if self.recorded_images < 1000:
            filename = f'image_{self.recorded_images:05d}.jpg'
            (self.record_dir / filename).write_bytes(observation['jpeg'])
            self.recorded_images += 1
            details = dict(observation)
            del details['jpeg']
            details['image'] = filename
            self._event('observation', details)
        if task.arguments['model'] == 'lightnav':
            self.model_future = self.pool.submit(
                self._lightnav_predict, task.id, observation['jpeg'], task.arguments['instruction']
            )
        else:
            request = self.foresight_type.Request()
            request.goal_command.data = task.arguments['instruction']
            self.model_future = self.foresight.call_async(request)
        self.next_prediction = time.monotonic() + self.cfg['model_period_s']

    def _in_evaluation_region(self, task):
        goal = task.arguments.get('evaluation_goal')
        if not goal:
            return None
        robot = self._pose(goal['frame'])
        distance = math.hypot(robot[0] - goal['x'], robot[1] - goal['y'])
        task.progress['evaluation_distance_m'] = distance
        return distance <= goal['tolerance']

    def _apply_prediction(self, result, context):
        if context['model'] == 'foresight':
            plan = result.foresight_plan
            rows = foresight_waypoints(plan)
            frame, stop = 'odom', False
            # Foresight plans from its own latest image, not from the manager's observation.
            stamp_s = stamp_seconds(plan.header.stamp)
            ros_now = self.get_clock().now().nanoseconds / 1e9
            if ros_now - stamp_s > self.cfg['prediction_max_age_s']:
                raise ValueError('Foresight planned from an expired image')
        else:
            frame, rows, stop = result['frame'], result['waypoints'], result['stop']
            stamp_s = context['observation']['stamp_s']
        self._event(
            'prediction',
            {'frame': frame, 'waypoints': rows, 'stop': stop, 'observation_stamp_s': stamp_s},
        )
        self.last_prediction = time.monotonic()
        if stop:
            # LightNav sets stop when its whole action chunk is zero; that is not a measurement.
            reached = self._in_evaluation_region(self.tasks.active)
            if reached is None:
                self._finish('succeeded', 'Model stop claim; destination not measured')
            elif reached:
                self._finish('succeeded', 'Configured destination region reached')
            else:
                self._finish('failed', 'Model stopped outside the evaluation region')
            return
        if self.nav_goal is not None or self.nav_pending:
            return
        subgoal = self._choose_subgoal(rows, frame, context['observation']['robot_pose'])
        if subgoal is None:
            raise ValueError('Model produced no feasible subgoal within the configured distance')
        x, y, yaw = subgoal
        self._send_goal(x, y, yaw, self.map_frame)

    def _choose_subgoal(self, rows, frame, observation_pose):
        """Return the farthest feasible waypoint as a map pose, or None."""
        robot = self._pose(self.map_frame)
        min_distance = self.cfg['min_subgoal_distance_m']
        max_distance = self.cfg['max_subgoal_distance_m']
        for row in reversed(rows):
            if len(row) not in (2, 3):
                raise ValueError('Waypoint must contain x, y and optional yaw')
            x = finite(row[0], 'waypoint.x')
            y = finite(row[1], 'waypoint.y')
            yaw = 0.0
            if len(row) == 3:
                yaw = finite(row[2], 'waypoint.yaw')
            goal_frame = frame
            if frame == 'base_link':
                # Relative waypoints belong to the pose where the image was taken.
                origin = (observation_pose['x'], observation_pose['y'], observation_pose['yaw'])
                x, y = transform_xy((x, y), origin)
                yaw += origin[2]
                goal_frame = 'odom'
            try:
                x, y, yaw = self._validate_goal(x, y, yaw, goal_frame)
            except ValueError:
                continue
            distance = math.hypot(x - robot[0], y - robot[1])
            if min_distance <= distance <= max_distance:
                return x, y, yaw
        return None

    def _poll_model(self):
        if self.model_future is None:
            return
        context = self.model_context
        if not self.model_future.done():
            if (
                self.tasks.current(context['id'], context['generation'])
                and time.monotonic() - context['created'] > self.cfg['model_timeout_s']
            ):
                self._finish('failed', 'Model inference timed out')
            return
        future, self.model_future = self.model_future, None
        if not self.tasks.current(context['id'], context['generation']):
            return
        try:
            if time.monotonic() - context['created'] > self.cfg['prediction_max_age_s']:
                raise ValueError('Prediction arrived after its observation expired')
            self._apply_prediction(future.result(), context)
        except Exception as exc:
            self._finish('failed', f'Model error: {exc}')

    def _handle_requests(self):
        for _ in range(8):
            try:
                name, arguments, future = self.requests.get_nowait()
            except queue.Empty:
                return
            if not future.set_running_or_notify_cancel():
                continue
            try:
                tool = getattr(self, 'tool_' + name, None)
                if tool is None:
                    raise ValueError('Unknown tool')
                future.set_result(tool(**arguments))
            except Exception as exc:
                future.set_result({'error': str(exc)})

    def _supervise_task(self):
        task = self.tasks.active
        if task is None:
            return
        if self.tasks.expired():
            self._finish('timed_out', 'Mission deadline exceeded')
            return
        problems = self._readiness(visual=task.kind == 'visual')
        if problems:
            self._finish('failed', ', '.join(problems))
            return
        if task.kind != 'visual':
            return
        try:
            if self._in_evaluation_region(task):
                self._finish('succeeded', 'Configured destination region reached')
                return
            if self.model_future is None and time.monotonic() >= self.next_prediction:
                self._request_prediction()
        except Exception as exc:
            self._finish('failed', str(exc))

    def _gate(self):
        """Return the command for /cmd_vel_nav and the reason the gate is open or closed."""
        task = self.tasks.active
        now = time.monotonic()
        if task is None:
            return Twist(), 'idle'
        if not self.autonomy_enabled:
            return Twist(), 'waiting_for_autonomy'
        if self.nav_goal is None:
            return Twist(), 'waiting_for_goal'
        if task.kind == 'visual':
            if self.last_prediction is None:
                return Twist(), 'prediction_expired'
            if now - self.last_prediction > self.cfg['prediction_max_age_s']:
                return Twist(), 'prediction_expired'
        if now - self.candidate_time > 0.5:
            return Twist(), 'nav_command_expired'
        return self.last_candidate, 'open'

    def _tick(self):
        self._handle_requests()
        self._poll_model()
        self._supervise_task()
        command, self.gate_reason = self._gate()
        self.output.publish(command)
        if self.gate_reason != self.previous_gate_reason:
            self._event('gate', {'reason': self.gate_reason})
            self.previous_gate_reason = self.gate_reason
        if time.monotonic() - self.last_state_log >= 0.5:
            state = self.tool_get_state()
            self.status_pub.publish(String(data=json.dumps(state, allow_nan=False)))
            self._event('state', state)
            self.last_state_log = time.monotonic()

    def tool_navigate_to_pose(self, x, y, yaw=0.0, frame='map', timeout_s=120.0):
        x, y, yaw = finite(x, 'x'), finite(y, 'y'), finite(yaw, 'yaw')
        self._validate_goal(x, y, yaw, frame)
        task = self._start('metric', {'x': x, 'y': y, 'yaw': yaw, 'frame': frame}, timeout_s)
        try:
            self._send_goal(x, y, yaw, frame)
        except Exception as exc:
            self._finish('failed', str(exc))
            raise
        return task.view(time.monotonic())

    def tool_start_visual_navigation(
        self,
        instruction,
        model='foresight',
        timeout_s=120.0,
        goal_x=None,
        goal_y=None,
        goal_frame='map',
        goal_tolerance=0.25,
    ):
        if not isinstance(instruction, str) or not instruction.strip() or len(instruction) > 2000:
            raise ValueError('Provide an instruction between 1 and 2000 characters')
        if model not in ('foresight', 'lightnav'):
            raise ValueError('model must be foresight or lightnav')
        if model == 'lightnav':
            if not self.cfg['lightnav_url']:
                raise ValueError('Configure lightnav_url to a real model server before starting')
            try:
                import_module('websockets.sync.client')
            except ImportError as exc:
                raise ValueError(
                    'Install the lightnav optional dependency in the manager environment'
                ) from exc
        else:
            try:
                client = self._foresight_client()
            except (ImportError, ModuleNotFoundError, AttributeError) as exc:
                raise ValueError(
                    'Build the upstream amrl_msgs interfaces and start the Foresight planner'
                ) from exc
            if not client.service_is_ready():
                raise ValueError('Foresight planner service is unavailable')
        args = {'instruction': instruction.strip(), 'model': model}
        if goal_x is not None or goal_y is not None:
            tolerance = finite(goal_tolerance, 'goal_tolerance')
            if not 0.05 <= tolerance <= 1.0 or goal_frame not in ('map', 'odom'):
                raise ValueError('Invalid evaluation goal frame or tolerance')
            args['evaluation_goal'] = {
                'x': finite(goal_x, 'goal_x'),
                'y': finite(goal_y, 'goal_y'),
                'frame': goal_frame,
                'tolerance': tolerance,
            }
        return self._start('visual', args, timeout_s).view(time.monotonic())

    def tool_get_task(self, task_id):
        task = self.tasks.items.get(task_id)
        if task is None:
            raise ValueError('Unknown task_id')
        return task.view(time.monotonic())

    def tool_cancel_task(self, task_id):
        task = self.tasks.items.get(task_id)
        if task is None:
            raise ValueError('Unknown task_id')
        if self.tasks.active is task:
            self._finish('cancelled', 'Requested by client')
        return task.view(time.monotonic())

    def tool_stop(self):
        task_id = None
        if self.tasks.active is not None:
            task_id = self.tasks.active.id
        self._finish('cancelled', 'Autonomous stop requested')
        return {'stopped': True, 'task_id': task_id, 'scope': 'autonomous_traction'}

    def close(self):
        self._finish('cancelled', 'Manager shutdown')
        self.server.shutdown()
        self.server.server_close()
        self.pool.shutdown(wait=False, cancel_futures=True)


def main():
    rclpy.init()
    node = Manager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
