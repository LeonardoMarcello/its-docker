#!/bin/bash
set -e

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# Production image: workspace baked into the image
if [ -f /opt/its_ws/install/setup.bash ]; then
  source /opt/its_ws/install/setup.bash
fi

# If a workspace overlay exists in the mounted workspace, source it
if [ -f "$HOME/ros2/install/setup.bash" ]; then
  source "$HOME/ros2/install/setup.bash"
fi

exec "$@"
