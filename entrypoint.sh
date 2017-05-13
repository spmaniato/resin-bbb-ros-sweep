#!/bin/bash
set -e

# Set up ROS environment
source "${ROS_INSTALL_DIR}/setup.bash"

# Set up ROS networking-related parameters
export ROS_HOSTNAME="${HOSTNAME}"
echo -e "127.0.1.1\t${HOSTNAME}" >> /etc/hosts

exec "$@"
