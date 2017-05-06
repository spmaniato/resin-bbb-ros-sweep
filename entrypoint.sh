#!/bin/bash
set -e

# Set up ROS environment
source "/opt/ros/${ROS_DISTRO}/setup.bash"

exec "$@"
