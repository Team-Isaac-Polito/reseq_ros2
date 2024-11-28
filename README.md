# Reseq_ros2

ROS2 code to control the ReseQ robot.

## ROS workspace setup

Inside the folder where you want to keep your ROS workspace, run the following commands to create the workspace and clone this repository

```bash
mkdir ros2_ws
cd ros2_ws
git clone --recursive https://github.com/Team-Isaac-Polito/reseq_ros2.git src
```

This will copy the contents of this repository inside the `ros2_ws/src` folder.

In VSCode (or your editor of choice) open `ros2_ws`

## Install dependencies with ROSDEP

In the directory `ros2_ws` with the apt cache updated run:

```bash
rosdep install -iy --from-path src --rosdistro humble
```

## Development dependencies

Install dependencies to manually inspect and test the environment:

```
sudo apt install -y can-utils ros-humble-rviz2 python3-flake8 python3-flake8-blind-except python3-flake8-builtins python3-flake8-class-newline python3-flake8-comprehensions python3-flake8-deprecated python3-flake8-docstrings python3-flake8-import-order python3-flake8-quotes curl lsb-release gnupg
```

## VSCode Development configuration

Install the following extensions:

```
code --install-extension ms-python.flake8
code --install-extension ms-python.isort
code --install-extension charliermarsh.ruff
```

In `ros_ws`, modify the file `.vscode/settings.json` by adding the following configuration:

```json
    "flake8.args": ["--config", "src/reseq_ros2/test/flake8.cfg"],
    "[python]": {
        "editor.defaultFormatter": "charliermarsh.ruff",
        "editor.formatOnSave": true,
        "editor.codeActionsOnSave": {
            "source.organizeImports": "explicit",
            "python.sortImports": "explicit"
        }
    },
    "ruff.format.args": [
        "--config=src/ruff.toml",
        "--line-length=99"
    ],
    "ruff.organizeImports": false
```

The files should format correctly on save

## Testing 

To execute ROS2 tests, run the following command which generates a more detailed output:

```
colcon test --event-handlers=console_direct+
```

## Gazebo Simulator
Due to problems with wsl, the project uses _Gazebo Harmonic_, which can be installed from the official Gazebo repositories.

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt install ros-humble-ros-gzharmonic
```
