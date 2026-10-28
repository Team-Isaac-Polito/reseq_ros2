# reseq_traction_autonomy

Autonomous traction for Rese.Q. A task manager takes goals from MCP tools or a local HTTP API, asks a navigation model for subgoals, and lets Nav2 drive. Its gate is the only publisher on `/cmd_vel_nav`, so every Nav2 command still passes through the existing mux and traction controllers. The arm is not touched.

```
MCP tools -> task manager -> navigation model -> subgoal -> Nav2 -> gate -> cmd_vel_mux -> traction
```

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select reseq_traction_autonomy --symlink-install
source install/setup.bash

python3 -m venv --system-site-packages "$HOME/.venvs/reseq-traction"
"$HOME/.venvs/reseq-traction/bin/pip" install -r src/reseq_traction_autonomy/requirements-mcp.txt
```

## Run in Gazebo

```bash
export ROS_DOMAIN_ID=73
ros2 launch reseq_traction_autonomy gazebo.launch.py \
  manager_python:="$HOME/.venvs/reseq-traction/bin/python"
```

The launch starts `simple_course.world`, the robot and app stack, SLAM, Nav2 and the manager. It leaves out the old `autonomy_coordinator`. Closing the Gazebo window shuts down the whole launch.

## Tools

```bash
ros2 run reseq_traction_autonomy traction_tool get_state
ros2 run reseq_traction_autonomy traction_tool observe
ros2 run reseq_traction_autonomy traction_tool navigate_to_pose \
  '{"x": 0.5, "y": 0.0, "frame": "map", "timeout_s": 30}'
ros2 run reseq_traction_autonomy traction_tool get_task '{"task_id": "TASK_ID"}'
ros2 run reseq_traction_autonomy traction_tool cancel_task '{"task_id": "TASK_ID"}'
ros2 run reseq_traction_autonomy traction_tool stop
```

`traction_tool` posts JSON to `http://127.0.0.1:8765/tools/TOOL_NAME`. The API only listens on localhost; forward port 8765 over SSH for remote use.

For an MCP client, run `bash src/reseq_traction_autonomy/scripts/run_mcp.sh`. It exposes the same seven tools with a `traction.` prefix.

## Navigation models

No weights ship with this package. If a model server is missing, the task fails with an error.

### LightNav-0

Start the [LightNav server](https://github.com/lightorigins/LightNav-0/blob/main/docs/DEPLOYMENT.md) on the GPU machine, forward its port, and pass the URL at launch:

```bash
ros2 launch reseq_traction_autonomy gazebo.launch.py \
  manager_python:="$HOME/.venvs/reseq-traction/bin/python" \
  lightnav_url:=ws://127.0.0.1:8050
```

Then call `start_visual_navigation` with `instruction` and `"model": "lightnav"`. Each task opens a fresh LightNav session. LightNav answers with a short list of `[forward, lateral, yaw]` poses relative to where the robot was when the image was taken. The manager places them in the odometry frame and sends the farthest reachable one (at most `max_subgoal_distance_m`) to Nav2.

LightNav's `stop` flag only means it predicted no further motion. The manager reports it as "Model stop claim; destination not measured".

### Foresight

Build `amrl_msgs` and `legged_deployment` from [foresight_public](https://github.com/ut-amrl/foresight_public) and set these planner parameters:

| Parameter | Value |
|---|---|
| `image_topic` | `/traction/camera/image/compressed` |
| `pose_topic` | `/odometry/filtered` |
| `path_frame_id` | `odom` |

The manager calls `/legged_deployment/foresight_planner`. It rejects paths not in `odom`, paths planned from an image older than `prediction_max_age_s`, and plans the critic rejected. The planner keeps its own image history and has no reset, so restart it between evaluation runs.

### Evaluation goal

Foresight never says it has arrived. For evaluation, `start_visual_navigation` on the HTTP API also accepts `goal_x`, `goal_y`, `goal_frame` and `goal_tolerance`. The model never sees them. The task succeeds when the SLAM pose enters that circle, and a LightNav stop outside it fails the task.

Model pages: [Foresight VLM](https://huggingface.co/ut-amrl/foresight-qwen3vl-2b-sft), [Foresight grounding policy](https://huggingface.co/ut-amrl/foresight-waypoint-policy), [LightNav-0](https://huggingface.co/LightOriginsHQ/LightNav-0).

## How a task runs

- One task at a time. Starting a second one fails until the first ends.
- Nav2's output goes to `/traction/nav2/candidate`. The gate forwards it to `/cmd_vel_nav` only while the task has an accepted Nav2 goal, a recent Nav2 command and, for visual tasks, a recent model answer.
- The manager publishes the robot outline, built from the three chassis transforms, to both Nav2 costmaps. Goals are checked against the map with the same outline.
- Stop, cancel, timeout and failure all send a zero command, cancel the Nav2 goal and switch autonomy off so the joystick works normally again.
- Moving the joystick past the mux deadband cancels the task.
- Sensor and model deadlines use wall time, so pausing Gazebo fails the task.
- Each task writes `events.jsonl` and up to 1,000 camera frames to `~/.local/state/reseq/traction/TASK_ID/`.

## Known issues

- With the real robot outline, Nav2 refuses some goals close to walls, usually when the tail would swing into them. With the old 0.12 m circle these goals were "reached" by ignoring the rear modules.
- Real model inference, destination accuracy and Jetson performance have not been tested yet.

## Tests

```bash
cd src/reseq_traction_autonomy
"$HOME/.venvs/reseq-traction/bin/python" -m pytest -q test
```

The LightNav tests use a local WebSocket server and run no model.
