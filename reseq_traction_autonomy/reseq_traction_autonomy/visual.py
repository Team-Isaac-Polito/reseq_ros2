"""Visual tasks: ask a navigation model where to go and turn its answer into Nav2 goals.

A visual task loops: send a camera frame, get waypoints back, pick the farthest
waypoint the robot can reach, and send it to Nav2 as a subgoal.
"""

import math
import time
from importlib import import_module

from .core import finite, transform_xy
from .models import foresight_waypoints
from .navigation import Navigation
from .ros_interface import stamp_seconds

MAX_RECORDED_IMAGES = 1000


class VisualNavigation(Navigation):
    # ---- Model requests ----

    def _model_context(self, kind, observation):
        """Remember what a model request was for, so a late reply can be checked."""
        task = self.tasks.active
        now = time.monotonic()
        # Keep a steady frame cadence; restart it only after a long gap such as a prediction.
        period = self.cfg['frame_period_s']
        due = self.next_frame + period
        if due > now:
            self.next_frame = due
        else:
            self.next_frame = now + period
        return {
            'kind': kind,
            'id': task.id,
            'generation': self.tasks.generation,
            'created': now,
            'observation': observation,
            'model': task.arguments['model'],
        }

    def _send_history_frame(self):
        """Stream a frame between predictions so LightNav sees its trained frame rate."""
        observation = self._observation()
        self.model_context = self._model_context('frame', observation)
        task_id = self.tasks.active.id
        jpeg = observation['jpeg']
        self.model_future = self.pool.submit(self.lightnav.observe, task_id, jpeg)

    def _record_observation(self, observation):
        """Save the image and its poses in the task record."""
        if self.recorded_images >= MAX_RECORDED_IMAGES:
            return
        filename = f'image_{self.recorded_images:05d}.jpg'
        (self.record_dir / filename).write_bytes(observation['jpeg'])
        self.recorded_images += 1
        details = dict(observation)
        del details['jpeg']
        details['image'] = filename
        self._event('observation', details)

    def _request_prediction(self):
        task = self.tasks.active
        observation = self._observation()
        self.model_context = self._model_context('prediction', observation)
        self._record_observation(observation)
        instruction = task.arguments['instruction']
        if task.arguments['model'] == 'lightnav':
            self.model_future = self.pool.submit(
                self.lightnav.predict, task.id, observation['jpeg'], instruction
            )
        else:
            request = self.foresight_type.Request()
            request.goal_command.data = instruction
            self.model_future = self.foresight.call_async(request)
        self.next_prediction = time.monotonic() + self.cfg['model_period_s']

    def _poll_model(self):
        """Handle a finished model request, or fail the task if it takes too long."""
        if self.model_future is None:
            return
        context = self.model_context
        still_current = self.tasks.current(context['id'], context['generation'])
        waited = time.monotonic() - context['created']
        if not self.model_future.done():
            if still_current and waited > self.cfg['model_timeout_s']:
                self._finish('failed', 'Model inference timed out')
            return
        future = self.model_future
        self.model_future = None
        if not still_current:
            return
        try:
            result = future.result()
            # History frames have no answer to act on.
            if context['kind'] == 'frame':
                return
            if waited > self.cfg['prediction_max_age_s']:
                raise ValueError('Prediction arrived after its observation expired')
            self._apply_prediction(result, context)
        except Exception as exc:
            self._finish('failed', f'Model error: {exc}')

    # ---- Predictions ----

    def _read_prediction(self, result, context):
        """Return (frame, waypoint rows, stop, image stamp) from either model's reply."""
        if context['model'] != 'foresight':
            stamp_s = context['observation']['stamp_s']
            return result['frame'], result['waypoints'], result['stop'], stamp_s
        plan = result.foresight_plan
        rows = foresight_waypoints(plan)
        # Foresight plans from its own latest image, not from the manager's observation.
        stamp_s = stamp_seconds(plan.header.stamp)
        ros_now = self.get_clock().now().nanoseconds / 1e9
        if ros_now - stamp_s > self.cfg['prediction_max_age_s']:
            raise ValueError('Foresight planned from an expired image')
        return 'odom', rows, False, stamp_s

    def _stop_claimed(self):
        """Count a stop answer; after a run of them, end the task."""
        # LightNav sets stop when its whole action chunk is zero; that is not a measurement.
        self.stop_claims += 1
        if self.stop_claims < self.cfg['stop_confirmations']:
            return
        reached = self._in_evaluation_region(self.tasks.active)
        if reached is None:
            self._finish('succeeded', 'Model stop claim; destination not measured')
        elif reached:
            self._finish('succeeded', 'Configured destination region reached')
        else:
            self._finish('failed', 'Model stopped outside the evaluation region')

    def _apply_prediction(self, result, context):
        frame, rows, stop, stamp_s = self._read_prediction(result, context)
        prediction = {
            'frame': frame,
            'waypoints': rows,
            'stop': stop,
            'observation_stamp_s': stamp_s,
        }
        self._event('prediction', prediction)
        self.last_prediction = time.monotonic()
        if stop:
            self._stop_claimed()
            return
        self.stop_claims = 0

        if self.nav_pending or self.backing_up:
            return
        driving = self.nav_goal is not None
        # Keep the model in charge: a newer prediction replaces a subgoal after a few seconds.
        if driving:
            subgoal_age = time.monotonic() - self.subgoal_sent
            if subgoal_age < self.cfg['subgoal_refresh_s']:
                return
        subgoal = self._choose_subgoal(rows, frame, context['observation']['robot_pose'])
        if subgoal is None:
            # Turn-in-place or blocked answers are common. Keep driving the current subgoal;
            # when there is none, a run of them backs the robot away from the wall.
            self._event('no_feasible_subgoal', {'waypoints': rows})
            if not driving:
                self._note_unreachable()
            return
        self.unreachable_predictions = 0
        x, y, yaw = subgoal
        self._send_goal(x, y, yaw, self.map_frame, replace=True)

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

    def _in_evaluation_region(self, task):
        """True/False if the task has an evaluation goal, else None."""
        goal = task.arguments.get('evaluation_goal')
        if not goal:
            return None
        robot = self._pose(goal['frame'])
        distance = math.hypot(robot[0] - goal['x'], robot[1] - goal['y'])
        task.progress['evaluation_distance_m'] = distance
        return distance <= goal['tolerance']

    # ---- Called by the manager ----

    def _drive_visual_task(self, task):
        """Each tick: finish at the goal, or send the next frame or prediction request."""
        if self._in_evaluation_region(task):
            self._finish('succeeded', 'Configured destination region reached')
            return
        if self.model_future is not None:
            return
        now = time.monotonic()
        if now >= self.next_prediction:
            self._request_prediction()
        elif task.arguments['model'] == 'lightnav' and now >= self.next_frame:
            self._send_history_frame()

    def _check_model_available(self, model):
        """Raise if the chosen model cannot be reached."""
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
            return
        try:
            client = self._foresight_client()
        except (ImportError, ModuleNotFoundError, AttributeError) as exc:
            raise ValueError(
                'Build the upstream amrl_msgs interfaces and start the Foresight planner'
            ) from exc
        if not client.service_is_ready():
            raise ValueError('Foresight planner service is unavailable')
