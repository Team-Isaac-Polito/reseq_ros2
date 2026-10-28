"""Stdio MCP frontend; ROS and model dependencies stay in separate processes."""

import base64
import json

from mcp.server.fastmcp import FastMCP, Image
from .client import call

server = FastMCP('Rese.Q traction')


@server.tool(name='traction.get_state')
def get_state() -> dict:
    """Read robot pose, motion, observation ages and navigation readiness."""
    return call('get_state')


@server.tool(name='traction.observe')
def observe():
    """Get a recent camera image and its observation-time robot and camera poses."""
    result = call('observe')
    jpeg = base64.b64decode(result.pop('jpeg_base64'))
    return [json.dumps(result), Image(data=jpeg, format='jpeg')]


@server.tool(name='traction.navigate_to_pose')
def navigate_to_pose(
    x: float, y: float, yaw: float = 0.0, frame: str = 'map', timeout_s: float = 120.0
) -> dict:
    """Start a metric Nav2 task. Returns a task ID, not a claim of arrival."""
    return call(
        'navigate_to_pose',
        {'x': x, 'y': y, 'yaw': yaw, 'frame': frame, 'timeout_s': timeout_s},
    )


@server.tool(name='traction.start_visual_navigation')
def start_visual_navigation(
    instruction: str, model: str = 'foresight', timeout_s: float = 120.0
) -> dict:
    """Start navigation using an available frozen Foresight or LightNav worker."""
    return call(
        'start_visual_navigation',
        {'instruction': instruction, 'model': model, 'timeout_s': timeout_s},
    )


@server.tool(name='traction.get_task')
def get_task(task_id: str) -> dict:
    """Read the status and failure reason of an existing task."""
    return call('get_task', {'task_id': task_id})


@server.tool(name='traction.cancel_task')
def cancel_task(task_id: str) -> dict:
    """Cancel the specified task, including pending navigation requests."""
    return call('cancel_task', {'task_id': task_id})


@server.tool(name='traction.stop')
def stop() -> dict:
    """Close the autonomous traction gate and cancel its current mission."""
    return call('stop')


def main():
    server.run(transport='stdio')


if __name__ == '__main__':
    main()
