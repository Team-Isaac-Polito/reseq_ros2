"""Adapters for frozen navigation models. No model weights are bundled."""

import base64
import json

from .core import yaw_of


class LightNav:
    def __init__(self, url, timeout):
        from websockets.sync.client import connect

        self.socket = connect(
            url, open_timeout=timeout, close_timeout=1, max_size=16 * 1024 * 1024
        )
        self.timeout = timeout
        self.seq = 0
        self._request({'action': 'login', 'data': {'clientId': 'reseq-traction'}})
        self._request({'action': 'reset', 'data': {}})

    def _request(self, payload):
        self.socket.send(json.dumps(payload))
        reply = json.loads(self.socket.recv(timeout=self.timeout))
        data = reply.get('data', {})
        if data.get('rc') != 0:
            raise RuntimeError(f'LightNav rejected request: {data.get("msg", reply)}')
        return data

    def predict(self, jpeg, instruction):
        self.seq += 1
        data = self._request(
            {
                'action': 'next',
                'data': {
                    'seq': self.seq,
                    'image': base64.b64encode(jpeg).decode(),
                    'instruction': instruction,
                },
            }
        )
        if data.get('stop'):
            return {'stop': True, 'waypoints': [], 'frame': 'base_link'}
        rows = data.get('actions', {}).get('actions', [])
        if not rows:
            raise RuntimeError('LightNav returned no actions')
        for row in rows:
            if len(row) != 3:
                raise RuntimeError('Expected LightNav rows of [forward_m, lateral_m, yaw_rad]')
        # Every row is a cumulative pose relative to the robot at this image.
        return {'stop': False, 'waypoints': rows, 'frame': 'base_link'}

    def close(self):
        self.socket.close()


def foresight_waypoints(plan):
    """Return [x, y, yaw] rows from an amrl_msgs ForesightPlannerMsg."""
    if not plan.verdict.data:
        raise ValueError('Foresight rejected its plan: ' + plan.reason.data)
    frame = plan.path.header.frame_id or plan.header.frame_id
    # The planner labels its odometry pose with path_frame_id; it does not look up TF.
    if frame != 'odom':
        raise ValueError('Foresight path_frame_id must be odom, matching its pose_topic')
    rows = []
    for pose in plan.path.poses:
        position = pose.pose.position
        rows.append([position.x, position.y, yaw_of(pose.pose.orientation)])
    return rows
