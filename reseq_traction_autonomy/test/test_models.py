import base64
from collections import deque
import json
import threading

import pytest

server_module = pytest.importorskip('websockets.sync.server')

from reseq_traction_autonomy.models import LightNav, LightNavSession


@pytest.fixture
def worker():
    received = []
    replies = deque()

    def handle(socket):
        for raw in socket:
            request = json.loads(raw)
            received.append(request)
            data = {'rc': 0}
            if request['action'] == 'next' and request['data']['instruction']:
                data = replies.popleft()
            elif request['action'] == 'next':
                data = {'rc': 0, 'msg': 'image received'}
            socket.send(json.dumps({'data': data}))

    with server_module.serve(handle, '127.0.0.1', 0) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = f'ws://127.0.0.1:{server.socket.getsockname()[1]}'
        yield url, received, replies
        server.shutdown()
        thread.join(timeout=2)


def test_lightnav_protocol_resets_history_and_returns_relative_chunk(worker):
    url, received, replies = worker
    replies.append({'rc': 0, 'actions': {'actions': [[0.5, 0.1, 0.2], [0.6, 0.0, 0.1]]}})
    replies.append({'rc': 0, 'stop': True})
    model = LightNav(url, timeout=2)
    try:
        prediction = model.predict(b'image bytes', 'Go to the door')
        assert prediction == {
            'stop': False,
            'waypoints': [[0.5, 0.1, 0.2], [0.6, 0.0, 0.1]],
            'frame': 'base_link',
        }
        assert model.predict(b'next image', 'Go to the door')['stop'] is True
    finally:
        model.close()
    assert [item['action'] for item in received] == ['login', 'reset', 'next', 'next']
    assert received[2]['data']['seq'] == 1
    assert received[3]['data']['seq'] == 2
    assert base64.b64decode(received[2]['data']['image']) == b'image bytes'
    assert received[2]['data']['instruction'] == 'Go to the door'


@pytest.mark.parametrize('reply', [{'rc': 1, 'msg': 'worker failed'}, {'rc': 0}])
def test_lightnav_rejects_worker_errors_and_empty_actions(worker, reply):
    url, _, replies = worker
    replies.append(reply)
    model = LightNav(url, timeout=2)
    try:
        with pytest.raises(RuntimeError):
            model.predict(b'image bytes', 'Go forward')
    finally:
        model.close()


def test_history_frames_are_sent_without_an_instruction(worker):
    url, received, replies = worker
    replies.append({'rc': 0, 'actions': {'actions': [[0.3, 0.0, 0.0]]}})
    model = LightNav(url, timeout=2)
    try:
        model.observe(b'frame 1')
        model.observe(b'frame 2')
        model.predict(b'frame 3', 'Go to the door')
    finally:
        model.close()
    frames = [item['data'] for item in received if item['action'] == 'next']
    assert [frame['instruction'] for frame in frames] == ['', '', 'Go to the door']
    assert [frame['seq'] for frame in frames] == [1, 2, 3]


def test_session_opens_a_fresh_connection_for_each_task(worker):
    url, received, _ = worker
    session = LightNavSession(url, timeout=2)
    try:
        session.observe('task-a', b'a1')
        session.observe('task-a', b'a2')
        session.observe('task-b', b'b1')
    finally:
        session.close()
    actions = [item['action'] for item in received]
    assert actions == ['login', 'reset', 'next', 'next', 'login', 'reset', 'next']
