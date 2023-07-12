#!/bin/bash
set -e

export ROS_DISTRO=humble
export COLCON_WS=/colcon_ws

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /ros2_ws/install/local_setup.bash" >> ~/.bashrc

# setup ros2 environment
source "/opt/ros/${ROS_DISTRO}/setup.bash" --
# source gazebo
source /usr/share/gazebo/setup.bash --
# source the colcon workspace
source "${COLCON_WS}/install/setup.bash" --

exec "$@"
