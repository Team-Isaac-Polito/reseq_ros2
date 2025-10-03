#!/bin/bash

##
## Run the audio capture as follows:
##
##

# Exit if any command fails
set -e

# Define workspace path
WORKSPACE=~/audio_ws
SRC_DIR=$WORKSPACE/src

# Create workspace directories
echo "[INFO] Creating workspace directory at $SRC_DIR..."
mkdir -p "$SRC_DIR"

# Move to source directory
cd "$SRC_DIR"

# Create the repos file
echo "[INFO] Creating audio.repos file..."
cat <<EOF > "$WORKSPACE/audio.repos"
repositories:
  audio_common:
    type: git
    url: https://github.com/knorth55/audio_common.git
    version: ros2-idl-bugfix
  rosidl:
    type: git
    url: https://github.com/ros2/rosidl.git
    version: 3.3.1
  rosidl_python:
    type: git
    url: https://github.com/knorth55/rosidl_python.git
    version: fix-141
EOF

# Check if vcs is installed
if ! command -v vcs &> /dev/null; then
    echo "[INFO] Installing python3-vcstools..."
    apt update
    apt install -y python3-vcstools
fi

# Import the repositories
echo "[INFO] Importing repositories with vcs..."
vcs import < "$WORKSPACE/audio.repos"

# Install dependencies
echo "[INFO] Installing dependencies with rosdep..."
cd "$SRC_DIR"
rosdep update
rosdep install --from-paths . -y -r -i

# Build the workspace
echo "[INFO] Building the workspace with colcon..."
cd "$WORKSPACE"
colcon build

# Source setup script in ros_entrypoint.sh
echo "[INFO] Sourcing workspace setup in /ros_entrypoint.sh..."
sed -i "\$i ros_source_env /audio_ws/install/setup.bash \n" /ros_entrypoint.sh


echo "[DONE] Workspace built and configured successfully!"