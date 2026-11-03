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

The launch starts `simple_course.world`, the robot and app stack, SLAM, Nav2 and the manager. It leaves out the old `autonomy_coordinator`.

`world:=rescue_maze.world` loads a dwelling-search maze modelled on the RoboCup Rescue arena: plywood walls, four rooms off a central corridor, hazmat placards at 60 cm, a victim doll, backpack, hard hat, fire extinguisher, propane tank, debris, and start and end zones. Passages are wider than the league's and the floor is flat, so a 1.3 m robot that does not reverse can get around. An overhead camera publishes `/overview/image` for recordings. Change the layout in `worlds/generate_rescue_maze.py` and rerun it; it rewrites the world and its textures.

Gazebo runs as a server, and the window is a separate process. Closing the window leaves the simulation running; reopen it with `gz sim -g`, or start without it using `gui:=false`.

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

## Evaluate against ground truth

`traction_eval` runs a course of tasks and measures each one with Gazebo's true robot pose, not with SLAM:

```bash
ros2 run reseq_traction_autonomy traction_eval            # default: config/simple_course.yaml
ros2 run reseq_traction_autonomy traction_eval my_course.yaml --legs forward
```

At the start it records how the SLAM map lines up with the Gazebo world. For each leg it reports the true distance to the goal, the SLAM estimate of that distance, SLAM drift, path length and time. Results, including the true trajectory, are saved as JSON under `~/.local/state/reseq/traction/eval/`.

A course file lists legs with a goal in the map frame. Visual legs also take `kind: visual`, `instruction` and `model`; their goal is used for measurement and as the evaluation region, and the model never sees it.

## Navigation models

No weights ship with this package. If a model server is missing, the task fails with an error.

### LightNav-0

Start the [LightNav server](https://github.com/lightorigins/LightNav-0/blob/main/docs/DEPLOYMENT.md) on the GPU machine, forward its port, and pass the URL at launch:

```bash
ros2 launch reseq_traction_autonomy gazebo.launch.py \
  manager_python:="$HOME/.venvs/reseq-traction/bin/python" \
  lightnav_url:=ws://127.0.0.1:8050
```

Then call `start_visual_navigation` with `instruction` and `"model": "lightnav"`. Each task opens a fresh LightNav session. The model is trained on frames 0.25 s apart, so between predictions (every `model_period_s`, 1 s) the manager streams camera frames every `frame_period_s` (0.25 s) without asking for a prediction. LightNav answers with a short list of `[forward, lateral, yaw]` poses relative to where the robot was when the image was taken. The manager places them in the odometry frame and sends the farthest reachable one (at most `max_subgoal_distance_m`) to Nav2. A newer prediction replaces the Nav2 subgoal after `subgoal_refresh_s` (3 s), so the model stays in charge instead of one stale goal. If no point is reachable, for example when the model only turns in place, the prediction is skipped; after `escape_after_unreachable` (2) such predictions or failed subgoals, Nav2's BackUp behavior reverses `backup_distance_m` (0.3 m) and the model looks again. A failed subgoal does not end a visual task.

LightNav's `stop` flag only means it predicted no further motion, and it sometimes appears for a single frame. A task ends only after `stop_confirmations` (3) stop predictions in a row, and the manager reports it as "Model stop claim; destination not measured".

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
- The traction stack turns right for positive `angular.z`, on the robot and in Gazebo, while Nav2 uses the ROS convention (positive turns left). The gate flips the sign (`traction_turn_sign`). Joystick driving does not pass through the gate and is unchanged.
- The manager publishes the robot outline, built from the three chassis transforms, to both Nav2 costmaps. Goals are checked against the map with the same outline. Small unknown gaps between lidar rays (`unknown_gap_m`) count as free; unexplored areas do not.
- `cloud_decimator` keeps every 8th pixel of the depth camera cloud (about 5,000 points instead of 307,000). Nav2's costmaps and collision monitor read the thinned cloud; at full size most clouds were lost in transport and the collision monitor kept stopping the robot.
- `config/nav2_overrides.yaml` holds the Nav2 changes for this robot on top of the shared `nav2_params.yaml`: no reversing, a longer lookahead, and the thinned cloud.
- Readiness checks that Nav2's lifecycle manager reports the stack as active, not only that the action server exists.
- Stop, cancel, timeout and failure all send a zero command, cancel the Nav2 goal and switch autonomy off so the joystick works normally again.
- Moving the joystick past the mux deadband cancels the task.
- Sensor and model deadlines use wall time, so pausing Gazebo fails the task.
- Each task writes `events.jsonl` and up to 1,000 camera frames to `~/.local/state/reseq/traction/TASK_ID/`.

## Known issues

- Steering responds late. The FTL controller bends the yaw joints so the rear modules follow the head's past path, and in Gazebo that bending outweighs the head's own tracks. After the steering direction changes, the robot keeps turning the old way for roughly one module length (about 4 s at 0.1 m/s). Nav2's controller assumes an immediate response, so tight turns swing wide.
- The wheel odometry heading is mirrored. The wheel model is mirrored to match the robot's turning convention, so `diff_controller1/odom` reports a left turn when the robot turns right, and the EKF follows it because it fuses the wheel x/y position. Only SLAM recovers the true heading. This probably affects the real robot too.
- With the real robot outline, Nav2 refuses goals where the tail would swing into a wall. With the old 0.12 m circle such goals were "reached" by ignoring the rear modules.
- `simple_course.world` is small for a 1.3 m robot. Straight legs reach the goal within 0.1 m; the turning leg ends about 0.3 m away, next to the east wall.
- Real model inference and Jetson performance have not been tested yet.

## Tests

```bash
cd src/reseq_traction_autonomy
"$HOME/.venvs/reseq-traction/bin/python" -m pytest -q test
```

The LightNav tests use a local WebSocket server and run no model.
