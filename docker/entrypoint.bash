#!/bin/bash
set -e

export ROS_DISTRO=foxy
export COLCON_WS=/colcon_ws

# setup ros2 environment
source "/opt/ros/${ROS_DISTRO}/setup.bash" --
# source gazebo
source /usr/share/gazebo/setup.bash --
# source the colcon workspace
source "${COLCON_WS}/install/setup.bash" --

exec "$@"
