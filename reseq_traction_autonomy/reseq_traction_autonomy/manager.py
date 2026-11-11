"""Traction mission lifecycle, navigation goals and model requests.

Every 50 ms _tick runs: handle API requests, read model replies,
supervise the active task, then publish the gated command.
Visual tasks (model requests and predictions) live in visual.py.
"""

import json
from pathlib import Path
import queue
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .core import finite
from .visual import VisualNavigation

MAX_INSTRUCTION_LENGTH = 2000
# The gate closes when Nav2 has not sent a command for this long.
COMMAND_MAX_AGE_S = 0.5


class Manager(VisualNavigation):

    # ---- Task start and end ----

    def _check_can_start(self, kind):
        """Raise if the robot or the launch setup is not ready for a task."""
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

    def _reset_task_state(self):
        self.last_candidate = Twist()
        self.candidate_time = 0.0
        self.last_prediction = None
        self.next_prediction = 0
        self.next_frame = 0
        self.stop_claims = 0
        self.unreachable_predictions = 0
        self.backing_up = False
        self.camera_lost_since = None
        self.qr_deadline = None
        self.qr_text = ''  # only codes read during this task count

    def _start(self, kind, arguments, timeout_s):
        self._check_can_start(kind)
        task = self.tasks.start(kind, arguments, timeout_s)
        try:
            self._open_record(task)
        except OSError:
            self.tasks.finish('failed', 'Could not create task record')
            raise
        self._reset_task_state()

        # Ask the mux to listen to the gate. The task fails if the mux refuses.
        task_id = task.id
        generation = self.tasks.generation

        def enabled(done):
            if not self.tasks.current(task_id, generation):
                return
            try:
                if done.result().success:
                    self.enabled_generation = generation
                else:
                    self._finish('failed', 'Autonomy enable rejected')
            except Exception as exc:
                self._finish('failed', str(exc))

        future = self.enable_client.call_async(SetBool.Request(data=True))
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
        """Append one event to the task record."""
        if not self.log_file:
            return
        record = {
            'event': kind,
            'wall_s': time.time(),
            'ros_s': self.get_clock().now().nanoseconds / 1e9,
            'data': data,
        }
        try:
            self.log_file.write(json.dumps(record, allow_nan=False) + '\n')
            self.log_file.flush()
        except OSError as exc:
            # A full disk must not stop the control loop; give up recording instead.
            self.get_logger().error(f'Task record stopped: {exc}')
            self.log_file = None

    def _cancel_model_request(self):
        if self.model_future is None:
            return
        if self.model_context['model'] == 'foresight':
            self.foresight.remove_pending_request(self.model_future)
        self.model_future.cancel()
        self.model_future = None
        self.model_context = None

    def _finish(self, status, reason):
        """End the active task (if any), stop the robot and give control back to teleop."""
        task = self.tasks.finish(status, reason)
        self.nav_epoch += 1
        self.nav_pending = False
        self._cancel_model_request()
        # Publishing fails once rclpy has shut down; the mux then stops on stale commands.
        if rclpy.ok():
            self.output.publish(Twist())
            if self.nav_goal is not None:
                self.nav_goal.cancel_goal_async()
            # Return the mux to teleop so joystick input below the override deadband works.
            was_enabled = task is not None or self.autonomy_enabled
            if was_enabled and self.enable_client.service_is_ready():
                self.enable_client.call_async(SetBool.Request(data=False))
        self.nav_goal = None
        if task:
            self._event('task_finished', task.view(time.monotonic()))
        if self.log_file:
            self.log_file.close()
            self.log_file = None
        self.gate_reason = 'idle'

    # ---- Every tick ----

    def _handle_requests(self):
        """Run up to 8 queued API tool calls on the ROS thread."""
        for _ in range(8):
            try:
                name, arguments, future = self.requests.get_nowait()
            except queue.Empty:
                return
            # The HTTP side may have given up already.
            if not future.set_running_or_notify_cancel():
                continue
            try:
                tool = getattr(self, 'tool_' + name, None)
                if tool is None:
                    raise ValueError('Unknown tool')
                future.set_result(tool(**arguments))
            except Exception as exc:
                future.set_result({'error': str(exc)})

    def _camera_problem_lasts(self):
        """Under load a frame or its transform can arrive late; fail only if it lasts."""
        now = time.monotonic()
        if self.camera_lost_since is None:
            self.camera_lost_since = now
        return now - self.camera_lost_since > self.cfg['camera_grace_s']

    def _supervise_task(self):
        task = self.tasks.active
        if task is None:
            return
        if self.tasks.expired():
            self._finish('timed_out', 'Mission deadline exceeded')
            return
        problems = self._readiness(visual=task.kind == 'visual')
        if problems == ['camera_not_ready']:
            if self._camera_problem_lasts():
                self._finish('failed', 'camera_not_ready')
            return
        self.camera_lost_since = None
        if problems:
            self._finish('failed', ', '.join(problems))
            return
        if task.kind == 'metric':
            self._check_zone_qr()
            return
        try:
            self._drive_visual_task(task)
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
        if now - self.candidate_time > COMMAND_MAX_AGE_S:
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
        # Publish and record the state twice a second.
        if time.monotonic() - self.last_state_log >= 0.5:
            state = self.tool_get_state()
            self.status_pub.publish(String(data=json.dumps(state, allow_nan=False)))
            self._event('state', state)
            self.last_state_log = time.monotonic()

    # ---- Tools (called through the local API) ----

    def tool_navigate_to_pose(
        self, x, y, yaw=0.0, frame='map', timeout_s=120.0, zone_qr=None
    ):
        x = finite(x, 'x')
        y = finite(y, 'y')
        yaw = finite(yaw, 'yaw')
        if zone_qr is not None:
            if not isinstance(zone_qr, str) or not zone_qr:
                raise ValueError('zone_qr must be the non-empty text of the QR code')
        self._validate_goal(x, y, yaw, frame)
        arguments = {'x': x, 'y': y, 'yaw': yaw, 'frame': frame, 'zone_qr': zone_qr}
        task = self._start('metric', arguments, timeout_s)
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
        length_error = f'Provide an instruction between 1 and {MAX_INSTRUCTION_LENGTH} characters'
        if not isinstance(instruction, str) or not instruction.strip():
            raise ValueError(length_error)
        if len(instruction) > MAX_INSTRUCTION_LENGTH:
            raise ValueError(length_error)
        self._check_model_available(model)
        arguments = {'instruction': instruction.strip(), 'model': model}
        # Optional: a measured destination, used by evaluations.
        if goal_x is not None or goal_y is not None:
            tolerance = finite(goal_tolerance, 'goal_tolerance')
            if tolerance < 0.05 or tolerance > 1.0:
                raise ValueError('Invalid evaluation goal frame or tolerance')
            if goal_frame not in ('map', 'odom'):
                raise ValueError('Invalid evaluation goal frame or tolerance')
            arguments['evaluation_goal'] = {
                'x': finite(goal_x, 'goal_x'),
                'y': finite(goal_y, 'goal_y'),
                'frame': goal_frame,
                'tolerance': tolerance,
            }
        task = self._start('visual', arguments, timeout_s)
        return task.view(time.monotonic())

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
