#!/bin/bash
set -e

# Set up ROS environment
source "${ROS_INSTALL_DIR}/setup.bash"

export ROS_HOSTNAME=localhost

exec "$@"
