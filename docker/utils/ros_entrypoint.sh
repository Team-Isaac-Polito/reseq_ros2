#!/bin/bash
#set -e

function ros_source_env()
{
        if [ -f "$1" ]; then
                echo "sourcing   $1"
                source "$1"
        else
                echo "notfound   $1"
        fi
}

ros_source_env "$ROS_ROOT/setup.bash"

echo "ROS_DISTRO $ROS_DISTRO"
echo "ROS_ROOT   $ROS_ROOT"

exec "$@"
