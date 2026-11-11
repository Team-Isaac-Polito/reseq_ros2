"""Nav2 goals for traction tasks: send, replace, back up and track their results.

Nav2's velocity command also arrives here, where it is limited and, in
reverse, re-steered for the tail before the gate may forward it.
"""

import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import BackUp, NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from std_msgs.msg import String
from tf2_ros import TransformException

from .core import clamp, line_pursuit_curvature
from .ros_interface import RosInterface

# Tail steering applies only while the tail's heading is within 60 degrees
# (cosine 0.5) of the goal line.
REVERSE_MIN_ALIGNMENT = 0.5


class Navigation(RosInterface):
    def __init__(self):
        super().__init__()
        self.backup = ActionClient(self, BackUp, '/backup')
        self.backing_up = False
        self.subgoal_sent = 0.0
        self.unreachable_predictions = 0
        self.qr_deadline = None
        # The computer_vision package reads QR codes and publishes their text.
        self.qr_text = ''
        self.create_subscription(String, self.cfg['qr_topic'], self._qr_cb, 10)
        self.create_subscription(Twist, self.cfg['candidate_topic'], self._candidate_cb, 5)

    def _qr_cb(self, message):
        self.qr_text = message.data

    # ---- Nav2's velocity command ----

    def _candidate_cb(self, message):
        """Nav2's command. Limit it and convert it to the traction stack's turn sign."""
        linear = message.linear.x
        angular = message.angular.z
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.last_candidate = Twist()
            self.candidate_time = 0.0
            return
        if linear < 0:
            angular = self._reverse_turn_rate(linear, angular)
        turn = self.cfg['traction_turn_sign'] * angular
        output = Twist()
        output.linear.x = clamp(linear, self.cfg['max_linear_speed'])
        output.angular.z = clamp(turn, self.cfg['max_angular_speed'])
        self.last_candidate = output
        self.candidate_time = time.monotonic()

    def _reverse_turn_rate(self, linear, nav2_turn):
        """Return the turn rate (ROS sign) for a reverse command.

        In reverse the tail leads and the traction stack applies the turn to the
        tail, but Nav2 computed it for base_link, which now comes last. Steering
        on that late signal makes the body swing wider on every leg. So steer
        the tail itself along the straight line through the goal along its yaw.
        """
        task = self.tasks.active
        following_goal = self.nav_goal is not None and not self.backing_up
        if task is None or task.subgoal is None or not following_goal:
            return nav2_turn
        tail_frame = f'chassis{int(self.cfg["num_modules"])}_link'
        try:
            x, y, yaw = self._pose(self.map_frame, source=tail_frame)
        except TransformException:
            return nav2_turn
        # Reversing, the tail travels opposite to the way the robot faces.
        travel = (x, y, yaw + math.pi)
        goal = task.subgoal
        line = (goal['x'], goal['y'], goal['yaw'] + math.pi)
        # A goal off to the side is left to Nav2.
        if math.cos(travel[2] - line[2]) < REVERSE_MIN_ALIGNMENT:
            return nav2_turn
        curvature = line_pursuit_curvature(travel, line, self.cfg['reverse_lookahead_m'])
        return abs(linear) * curvature

    # ---- Goals ----

    def _dispatch(self, client, goal, on_finished, on_rejected, on_feedback=None):
        """Send an action goal for the active task. Replies from older goals are ignored."""
        self.nav_pending = True
        self.nav_epoch += 1
        task_id = self.tasks.active.id
        generation = self.tasks.generation
        epoch = self.nav_epoch

        def current():
            """True while this goal still belongs to the running task."""
            if not self.tasks.current(task_id, generation):
                return False
            return self.nav_epoch == epoch

        def feedback(message):
            if on_feedback is not None and current():
                on_feedback(message.feedback)

        def accepted(done):
            handle = done.result()
            if not current():
                # Too late: cancel a goal nobody waits for.
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            self.nav_pending = False
            if not handle.accepted:
                on_rejected()
                return
            self.nav_goal = handle
            handle.get_result_async().add_done_callback(finished)

        def finished(done):
            if current():
                self.nav_goal = None
                on_finished(done.result().status)

        sent = client.send_goal_async(goal, feedback_callback=feedback)
        sent.add_done_callback(accepted)

    def _send_goal(self, x, y, yaw, frame, replace=False):
        """Send a NavigateToPose goal. With replace, Nav2 preempts the goal it is driving."""
        task = self.tasks.active
        if task is None or self.nav_pending:
            return
        if self.nav_goal is not None and not replace:
            return
        x, y, yaw = self._validate_goal(x, y, yaw, frame)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        # A yaw-only quaternion.
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        task.subgoal = {'x': x, 'y': y, 'yaw': yaw, 'frame': self.map_frame}
        self.subgoal_sent = time.monotonic()
        self._dispatch(
            self.nav,
            goal,
            self._goal_finished,
            self._goal_rejected,
            self._goal_feedback,
        )
        self._event('subgoal_sent', task.subgoal)

    def _goal_rejected(self):
        self._finish('failed', 'Nav2 rejected the goal')

    def _goal_feedback(self, feedback):
        self.tasks.active.progress['distance_remaining_m'] = feedback.distance_remaining

    def _goal_finished(self, status):
        task = self.tasks.active
        succeeded = status == GoalStatus.STATUS_SUCCEEDED
        if task.kind == 'metric':
            self._metric_goal_finished(task, succeeded, status)
            return

        # Visual task: one subgoal ended; the next prediction picks a new one.
        task.subgoal = None
        self.next_prediction = 0
        if succeeded:
            completed = task.progress.get('completed_subgoals', 0)
            task.progress['completed_subgoals'] = completed + 1
            self._event('subgoal_reached', task.progress)
            return
        # Near walls Nav2 often refuses a subgoal; let the model choose again instead of failing.
        self._event('subgoal_failed', {'status': status})
        self._note_unreachable()

    def _metric_goal_finished(self, task, succeeded, status):
        if not succeeded:
            self._finish('failed', f'Nav2 ended with status {status}')
            return
        zone_qr = task.arguments.get('zone_qr')
        if zone_qr:
            # Arrived. Succeed only once the zone's QR code is read (see _check_zone_qr).
            self.qr_deadline = time.monotonic() + self.cfg['zone_qr_timeout_s']
            self._event('reading_zone_qr', {'expected': zone_qr})
            return
        self._finish('succeeded', 'Nav2 reported success; position not independently measured')

    def _check_zone_qr(self):
        """At the goal, wait for the zone's QR code until it is read or time runs out."""
        if self.qr_deadline is None:
            return
        expected = self.tasks.active.arguments['zone_qr']
        if self.qr_text == expected:
            self._finish('succeeded', f'Reached the zone and read its QR code "{expected}"')
        elif time.monotonic() > self.qr_deadline:
            self._finish('failed', f'Reached the goal but did not read QR code "{expected}"')

    def _note_unreachable(self):
        """Back up after a run of predictions the robot cannot follow."""
        self.unreachable_predictions += 1
        if self.unreachable_predictions >= self.cfg['escape_after_unreachable']:
            self.unreachable_predictions = 0
            self._back_up()

    def _back_up(self):
        """Reverse a short distance with Nav2's collision-checked BackUp behavior."""
        if self.backing_up or self.nav_pending:
            return
        if not self.backup.server_is_ready():
            return
        goal = BackUp.Goal()
        goal.target.x = float(self.cfg['backup_distance_m'])
        goal.speed = float(self.cfg['backup_speed'])
        goal.time_allowance = Duration(seconds=10.0).to_msg()
        self.backing_up = True
        self._dispatch(self.backup, goal, self._backup_finished, self._backup_rejected)
        self._event('backup_sent', {'distance_m': goal.target.x})

    def _backup_rejected(self):
        self._backup_finished('rejected')

    def _backup_finished(self, status):
        self.backing_up = False
        self.next_prediction = 0
        self._event('backup_finished', {'status': status})
