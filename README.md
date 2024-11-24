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

## Install dependencies with ROSDEP

In the directory `ros2_ws` with the apt cache updated run:
```bash
rosdep install -iy --from-path src --rosdistro humble
```

## Development dependencies
Install dependencies to manually inspect and test the environment:
```
sudo apt install -y can-utils ros-humble-rviz2 ros-humble-ros-gzharmonic python3-flake8 python3-flake8-blind-except python3-flake8-builtins python3-flake8-class-newline python3-flake8-comprehensions python3-flake8-deprecated python3-flake8-docstrings python3-flake8-import-order python3-flake8-quotes
```