"""Local HTTP API: POST /tools/<name> with a JSON object runs manager.tool_<name>.

The HTTP thread only queues the request. The ROS thread runs the tool
(see Manager._handle_requests) and fills in the future with the result.
"""

from concurrent.futures import Future, TimeoutError
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import queue

MAX_BODY_BYTES = 8192
REPLY_TIMEOUT_S = 8


def make_server(manager, port):
    """Return an HTTP server on 127.0.0.1 that forwards tool calls to manager."""

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass  # keep the ROS log clean

        def do_POST(self):
            # Browsers send Origin. Refuse them so web pages cannot drive the robot.
            content_type = self.headers.get('Content-Type', '').split(';')[0]
            if self.headers.get('Origin') or content_type != 'application/json':
                self._reply(403, {'error': 'Local JSON clients only'})
                return
            try:
                tool, arguments = self._read_request()
                result = self._run(tool, arguments)
                self._reply(200, result)
            except Exception as exc:
                self._reply(400, {'error': str(exc)})

        def _read_request(self):
            length = int(self.headers.get('Content-Length', '0'))
            if length <= 0 or length > MAX_BODY_BYTES:
                raise ValueError(f'Request body must be between 1 and {MAX_BODY_BYTES} bytes')
            arguments = json.loads(self.rfile.read(length))
            if not isinstance(arguments, dict):
                raise ValueError('Arguments must be an object')
            if not self.path.startswith('/tools/'):
                raise ValueError('Unknown endpoint')
            tool = self.path.removeprefix('/tools/')
            return tool, arguments

        def _run(self, tool, arguments):
            future = Future()
            try:
                manager.requests.put_nowait((tool, arguments, future))
            except queue.Full:
                raise ValueError('Manager request queue is full')
            try:
                return future.result(timeout=REPLY_TIMEOUT_S)
            except TimeoutError:
                future.cancel()
                raise ValueError('ROS executor did not handle the request in time')

        def _reply(self, code, payload):
            body = json.dumps(payload, allow_nan=False).encode()
            try:
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass  # the client went away

    return ThreadingHTTPServer(('127.0.0.1', port), Handler)
