#!/usr/bin/env python3

from __future__ import annotations

import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmStateBridge(Node):
    def __init__(self):
        super().__init__('arm_state_bridge')

        self.declare_parameter('source_topic', '/joint_states')
        self.declare_parameter('output_mode', 'joint_state')
        self.declare_parameter('output_topic', '/arm_joint_states')
        self.declare_parameter('trajectory_topic', '/mk2_arm_controller/joint_trajectory')
        self.declare_parameter('trajectory_duration_sec', 0.1)
        self.declare_parameter('startup_positions', [float('nan')] * 6)
        self.declare_parameter('startup_position_tolerance', 0.02)
        self.declare_parameter('startup_hold_sec', 1.0)
        self.declare_parameter(
            'joint_names',
            [
                'mod1__base_pitch_arm_joint',
                'mod1__base_roll_arm_joint',
                'mod1__elbow_pitch_arm_joint',
                'mod1__forearm_roll_arm_joint',
                'mod1__wrist_pitch_arm_joint',
                'mod1__wrist_roll_arm_joint',
            ],
        )

        self._source_topic = self.get_parameter('source_topic').value
        self._output_mode = self.get_parameter('output_mode').value
        self._output_topic = self.get_parameter('output_topic').value
        self._trajectory_topic = self.get_parameter('trajectory_topic').value
        self._trajectory_duration_sec = self.get_parameter('trajectory_duration_sec').value
        startup_positions = list(self.get_parameter('startup_positions').value)
        self._startup_position_tolerance = float(
            self.get_parameter('startup_position_tolerance').value
        )
        self._startup_hold_sec = float(self.get_parameter('startup_hold_sec').value)
        self._joint_names = list(self.get_parameter('joint_names').value)
        self._startup_positions = None
        if len(startup_positions) == len(self._joint_names):
            startup_is_configured = any(
                math.isfinite(float(value)) for value in startup_positions
            )
            if startup_is_configured:
                self._startup_positions = [float(value) for value in startup_positions]
        self._startup_complete = self._startup_positions is None
        self._startup_started_at: float | None = None
        output_topic = (
            self._output_topic if self._output_mode == 'joint_state' else self._trajectory_topic
        )

        self._joint_state_pub = None
        self._trajectory_pub = None
        if self._output_mode == 'joint_state':
            self._joint_state_pub = self.create_publisher(JointState, self._output_topic, 10)
        elif self._output_mode == 'trajectory':
            self._trajectory_pub = self.create_publisher(
                JointTrajectory, self._trajectory_topic, 10
            )
        else:
            raise ValueError("output_mode must be 'joint_state' or 'trajectory'")

        self.create_subscription(JointState, self._source_topic, self._joint_state_callback, 10)

        self.get_logger().info(
            f'ArmStateBridge ready | source={self._source_topic} | mode={self._output_mode} | '
            f'output={output_topic}'
        )

    def _filter_joint_state(self, msg: JointState) -> JointState | None:
        name_to_index = {name: index for index, name in enumerate(msg.name)}
        missing = [name for name in self._joint_names if name not in name_to_index]
        if missing:
            return None

        filtered = JointState()
        filtered.header = msg.header
        filtered.name = list(self._joint_names)
        filtered.position = [msg.position[name_to_index[name]] for name in self._joint_names]

        if msg.velocity:
            filtered.velocity = [msg.velocity[name_to_index[name]] for name in self._joint_names]
        if msg.effort:
            filtered.effort = [msg.effort[name_to_index[name]] for name in self._joint_names]

        return filtered

    def _joint_state_callback(self, msg: JointState):
        filtered = self._filter_joint_state(msg)
        if filtered is None:
            return

        if self._output_mode == 'joint_state':
            self._joint_state_pub.publish(filtered)
            return

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = 'arm_base_link'
        traj.joint_names = list(self._joint_names)

        pt_start = JointTrajectoryPoint()
        pt_start.positions = list(filtered.position)
        pt_start.velocities = [0.0] * len(self._joint_names)
        pt_start.time_from_start = Duration(nanoseconds=0).to_msg()

        pt_target = JointTrajectoryPoint()
        target_positions = list(filtered.position)
        if not self._startup_complete:
            if self._startup_started_at is None:
                self._startup_started_at = time.monotonic()
            startup_error = max(
                abs(filtered.position[index] - self._startup_positions[index])
                for index in range(len(self._joint_names))
            )
            startup_hold_elapsed = (time.monotonic() - self._startup_started_at) >= self._startup_hold_sec
            if (not startup_hold_elapsed) or startup_error > self._startup_position_tolerance:
                target_positions = list(self._startup_positions)
            else:
                self._startup_complete = True

        pt_target.positions = target_positions
        pt_target.velocities = [0.0] * len(self._joint_names)
        pt_target.time_from_start = Duration(seconds=float(self._trajectory_duration_sec)).to_msg()

        traj.points = [pt_start, pt_target]
        self._trajectory_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = ArmStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
