"""Nav2 goals for traction tasks: send, replace, back up and track their results."""

import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import BackUp, NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration

from .ros_interface import RosInterface


class Navigation(RosInterface):
    def __init__(self):
        super().__init__()
        self.backup = ActionClient(self, BackUp, '/backup')
        self.backing_up = False
        self.subgoal_sent = 0.0
        self.unreachable_predictions = 0

    def _dispatch(self, client, goal, on_finished, on_rejected, on_feedback=None):
        """Send an action goal for the active task. Replies from older goals are ignored."""
        task = self.tasks.active
        self.nav_pending = True
        self.nav_epoch += 1
        task_id, generation, epoch = task.id, self.tasks.generation, self.nav_epoch

        def current():
            return self.tasks.current(task_id, generation) and self.nav_epoch == epoch

        def feedback(message):
            if on_feedback is not None and current():
                on_feedback(message.feedback)

        def accepted(done):
            handle = done.result()
            if not current():
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

        client.send_goal_async(goal, feedback_callback=feedback).add_done_callback(accepted)

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
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        task.subgoal = {'x': x, 'y': y, 'yaw': yaw, 'frame': self.map_frame}
        self.subgoal_sent = time.monotonic()
        self._dispatch(
            self.nav,
            goal,
            self._goal_finished,
            lambda: self._finish('failed', 'Nav2 rejected the goal'),
            self._goal_feedback,
        )
        self._event('subgoal_sent', task.subgoal)

    def _goal_feedback(self, feedback):
        self.tasks.active.progress['distance_remaining_m'] = feedback.distance_remaining

    def _goal_finished(self, status):
        task = self.tasks.active
        succeeded = status == GoalStatus.STATUS_SUCCEEDED
        if task.kind == 'metric':
            if succeeded:
                reason = 'Nav2 reported success; position not independently measured'
                self._finish('succeeded', reason)
            else:
                self._finish('failed', f'Nav2 ended with status {status}')
            return
        task.subgoal = None
        self.next_prediction = 0
        if succeeded:
            task.progress['completed_subgoals'] = task.progress.get('completed_subgoals', 0) + 1
            self._event('subgoal_reached', task.progress)
            return
        # Near walls Nav2 often refuses a subgoal; let the model choose again instead of failing.
        self._event('subgoal_failed', {'status': status})
        self._note_unreachable()

    def _note_unreachable(self):
        """Back up after a run of predictions the robot cannot follow."""
        self.unreachable_predictions += 1
        if self.unreachable_predictions >= self.cfg['escape_after_unreachable']:
            self.unreachable_predictions = 0
            self._back_up()

    def _back_up(self):
        """Reverse a short distance with Nav2's collision-checked BackUp behavior."""
        if self.backing_up or self.nav_pending or not self.backup.server_is_ready():
            return
        goal = BackUp.Goal()
        goal.target.x = float(self.cfg['backup_distance_m'])
        goal.speed = float(self.cfg['backup_speed'])
        goal.time_allowance = Duration(seconds=10.0).to_msg()
        self.backing_up = True
        self._dispatch(
            self.backup, goal, self._backup_finished, lambda: self._backup_finished('rejected')
        )
        self._event('backup_sent', {'distance_m': goal.target.x})

    def _backup_finished(self, status):
        self.backing_up = False
        self.next_prediction = 0
        self._event('backup_finished', {'status': status})
