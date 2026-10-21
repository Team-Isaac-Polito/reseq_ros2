[![ROS2 CI/CD](https://github.com/Team-Isaac-Polito/reseq_ros2/actions/workflows/ros-ci.yaml/badge.svg)](https://github.com/Team-Isaac-Polito/reseq_ros2/actions/workflows/ros-ci.yaml)

# reseq_ros2

ROS2 code to control the Rese.Q robot.

## ROS Workspace Setup

Inside the folder where you want to keep your ROS workspace, run the following commands to create the workspace and clone this repository:

```bash
mkdir ros2_ws
cd ros2_ws
git clone --recursive https://github.com/Team-Isaac-Polito/reseq_ros2.git src
```

This will copy the contents of this repository inside the `ros2_ws/src` folder.

To ensure the submodules (`reseq_interfaces`, `computer_vision`) are on the latest version, run the following command inside `ros2_ws`:

```bash
cd src && git submodule update --recursive --remote && cd ..
```

In VSCode (or your editor of choice) open `ros2_ws`.

## Install Dependencies with ROSDEP

In the directory `ros2_ws` with the apt cache updated run:

On Humble:
```bash
rosdep install -iy --from-path src --rosdistro humble
```
On Jazzy (fix buggy version of rplidar):
```bash
vcs import src < src/rplidar.repos
rosdep install -iy --from-path src --rosdistro jazzy
```

## Development dependencies

Install dependencies to manually inspect and test the environment:

```bash
sudo apt install -y can-utils ros-humble-rviz2 python3-flake8 python3-flake8-blind-except python3-flake8-builtins python3-flake8-class-newline python3-flake8-comprehensions python3-flake8-deprecated python3-flake8-docstrings python3-flake8-import-order python3-flake8-quotes curl lsb-release gnupg
```

## VSCode Development Configuration

### 1. Install Recommended Extensions

Install the following extensions:

```
code --install-extension ms-python.flake8
code --install-extension ms-python.isort
code --install-extension charliermarsh.ruff
```

### 2. Configure Workspace Settings

Add the following to `.vscode/settings.json` in your workspace root (`ros2_ws/`):

```json
{
    "flake8.args": ["--config", "src/reseq_ros2/test/flake8.cfg"],
    "[python]": {
        "editor.defaultFormatter": "charliermarsh.ruff",
        "editor.formatOnSave": true,
        "editor.codeActionsOnSave": {
            "source.organizeImports": "explicit",
            "python.sortImports": "explicit"
        }
    },
    "ruff.configuration": "${workspaceFolder}/src/ruff.toml",
    "ruff.organizeImports": false,
    "C_Cpp.intelliSenseEngine": "disabled",
    "clangd.arguments": [
        "--background-index",
        "--clang-tidy",
        "--completion-style=detailed",
        "--header-insertion=iwyu"
    ],
    "files.associations": {
        "*.h": "cpp",
        "*.hpp": "cpp"
    }
}
```

The files should format correctly on save

## Testing

To execute ROS2 tests, run the following command which generates a more detailed output:

```bash
colcon test --event-handlers=console_direct+
```

---

## Gazebo Simulation

We use **Gazebo Harmonic (gz-sim)** with **ROS 2 Jazzy** via `ros_gz_sim`. The simulation runs in a separate Docker container so it doesn't conflict with the Humble robot workspace.

### Quick Start (Docker)

From the `ros2_ws` folder, run the following commands to build and run the Gazebo simulation:

```bash
docker build -t reseq-gazebo -f src/docker/Dockerfile.gazebo src
```

Run the simulation for native Linux with:

```bash
xhost +local:docker
docker run --rm --net=host --privileged \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -e GAZEBO_WORLD=simple_course.world \
  -e LAUNCH_SIM=true -e LAUNCH_RVIZ=false \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -it reseq-gazebo
```

or WSL with:

```bash
docker run --rm --net=host --privileged \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -e GAZEBO_WORLD=simple_course.world \
  -e LAUNCH_SIM=true -e LAUNCH_RVIZ=false \
  -v /mnt/wslg/.X11-unix/X0:/tmp/.X11-unix/X0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -it reseq-gazebo
```

---

## Project Structure

```
src/
├── reseq_ros2/           # Core Python nodes (ftl_controller, scaler, autonomy_coordinator, detector, ...)
├── reseq_description/    # URDF/xacro, configs, ros2_control descriptions
├── reseq_hardware/       # C++ ros2_control hardware interface (CAN bus)
├── reseq_arm_mk2/        # Cartesian arm control for MK2
├── reseq_sim/            # Gazebo simulation (worlds, launch, arena models)
├── reseq_interfaces/     # Custom ROS 2 messages/services (submodule)
├── computer_vision/      # CV pipeline (submodule)
├── is_slam/              # SLAM utilities (is_geotiff, is_map_tools, is_nav_msgs)
└── docker/               # Dockerfiles (gazebo, orin)
```
