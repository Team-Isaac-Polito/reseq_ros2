#!/bin/bash

LAUNCH_CV=false
LAUNCH_APP=false

while [[ "$#" -gt 0 ]]; do
    case $1 in
        cv|--cv) LAUNCH_CV=true ;;
        app|--app) LAUNCH_APP=true ;;
        *) echo "Parametro sconosciuto: $1" ;;
    esac
    shift
done

gnome-terminal --tab --title="ROS2 Core" -- bash -c \
"source /opt/ros/jazzy/setup.bash; source ./install/local_setup.bash; \
ros2 launch reseq_ros2 reseq_launch.py config_file:=reseq_mk2_can.yaml; exec bash"

gnome-terminal --tab --title="Rosbridge" -- bash -c \
"source /opt/ros/jazzy/setup.bash; \
ros2 launch rosbridge_server rosbridge_websocket_launch.xml; exec bash"

if [ "$LAUNCH_CV" = true ]; then
    gnome-terminal --tab --title="Computer Vision" -- bash -c \
    "source /opt/ros/jazzy/setup.bash; cd ~/Documents/ros2_ws; source ./install/local_setup.bash; \
    ros2 launch computer_vision cv_launch.py mode:=3; exec bash"
fi

if [ "$LAUNCH_APP" = true ]; then
   gnome-terminal --tab --title="App Gateway" -- bash -c \
   "source /opt/ros/jazzy/setup.bash; cd ~/Documents/ros2_ws; source ./install/local_setup.bash;  \
    cd ~/Documents/ros2_ws ;ros2 run reseq_ros2 app_gateway; exec bash "
fi