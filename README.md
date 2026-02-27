[![ROS2 CI/CD](https://github.com/Team-Isaac-Polito/reseq_ros2/actions/workflows/ros-ci.yaml/badge.svg)](https://github.com/Team-Isaac-Polito/reseq_ros2/actions/workflows/ros-ci.yaml)

# reseq_ros2

ROS2 code to control the ReseQ robot.

## ROS workspace setup

Inside the folder where you want to keep your ROS workspace, run the following commands to create the workspace and clone this repository

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

In VSCode (or your editor of choice) open `ros2_ws`

## Install dependencies with ROSDEP

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

Then, from `ros2_ws`, configure the workspace linter settings:

```bash
python3 -c "
import json, os
path = '.vscode/settings.json'
os.makedirs('.vscode', exist_ok=True)
settings = json.load(open(path)) if os.path.exists(path) else {}
settings.update({
    'flake8.args': ['--config', 'src/reseq_ros2/test/flake8.cfg'],
    '[python]': {
        'editor.defaultFormatter': 'charliermarsh.ruff',
        'editor.formatOnSave': True,
        'editor.codeActionsOnSave': {
            'source.organizeImports': 'explicit',
            'python.sortImports': 'explicit',
        },
    },
    'ruff.configuration': '\${workspaceFolder}/src/ruff.toml',
    'ruff.organizeImports': False,
})
json.dump(settings, open(path, 'w'), indent=4)
print('Settings updated:', path)
"
```

The files should format correctly on save

## Testing 

To execute ROS2 tests, run the following command which generates a more detailed output:

```
colcon test --event-handlers=console_direct+
```
