from __future__ import annotations

import argparse
import json
import traceback
from pathlib import Path as FilesystemPath

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


def parse_waypoints(payload: object) -> list[tuple[float, float]]:
    if isinstance(payload, dict) and 'poses' in payload:
        payload = payload['poses']

    if not isinstance(payload, list):
        raise ValueError('Expected a list of waypoints or a nav_msgs/Path-like JSON payload.')

    waypoints = []
    for item in payload:
        if not isinstance(item, dict):
            raise ValueError('Each waypoint must be a JSON object.')

        if 'pose' in item:
            position = item['pose']['position']
            x_pos = position['x']
            y_pos = position['y']
        else:
            x_pos = item['x']
            y_pos = item['y']

        waypoints.append((float(x_pos), float(y_pos)))

    return waypoints


class UserPathPublisher(Node):
    def __init__(self, waypoints: list[tuple[float, float]], frame_id: str, publish_count: int):
        super().__init__('user_path_publisher')
        self.waypoints = waypoints
        self.frame_id = frame_id
        self.publish_count = publish_count
        self.publish_iteration = 0
        self.publisher = self.create_publisher(Path, '/autonomy/user_path', 10)
        self.timer = self.create_timer(0.5, self.publish_once)

    def publish_once(self) -> None:
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        for x_pos, y_pos in self.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x_pos
            pose.pose.position.y = y_pos
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.publisher.publish(path)
        self.publish_iteration += 1
        self.get_logger().info(
            f'Published {len(self.waypoints)} user-path waypoints to /autonomy/user_path '
            f'({self.publish_iteration}/{self.publish_count}).'
        )

        if self.publish_iteration >= self.publish_count:
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()


def load_payload(args: argparse.Namespace) -> object:
    if args.json_file:
        return json.loads(FilesystemPath(args.json_file).read_text())
    return json.loads(args.json)


def main(args=None):
    parser = argparse.ArgumentParser(description='Publish a user path to /autonomy/user_path.')
    parser.add_argument('--json', help='Inline JSON waypoint list or nav_msgs/Path-like payload.')
    parser.add_argument('--json-file', help='Path to a JSON file exported from the Flutter page.')
    parser.add_argument('--frame-id', default='map', help='Frame id for the published path.')
    parser.add_argument(
        '--publish-count',
        type=int,
        default=5,
        help='How many times to publish the path before exiting.',
    )
    parsed_args = parser.parse_args(args=args)

    if not parsed_args.json and not parsed_args.json_file:
        parser.error('Provide either --json or --json-file.')

    rclpy.init(args=None)
    try:
        payload = load_payload(parsed_args)
        waypoints = parse_waypoints(payload)
        if not waypoints:
            raise ValueError('The provided path is empty.')
        node = UserPathPublisher(waypoints, parsed_args.frame_id, parsed_args.publish_count)
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.logging.get_logger('user_path_publisher').warn(
            'UserPathPublisher interrupted by user'
        )
    except Exception as err:
        rclpy.logging.get_logger('user_path_publisher').fatal(
            f'Error in the user_path_publisher node: {str(err)}\n{traceback.format_exc()}'
        )
        rclpy.shutdown()


if __name__ == '__main__':
    main()
