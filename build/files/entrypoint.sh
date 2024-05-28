#!/bin/bash
set -e

# setup ros environment, this is copied from the entrypoint script of the normal ROS Dockerfile
source "/opt/ros/$ROS_DISTRO/setup.bash" --
# also, source our workspace
source "/ROS_WORKSPACE/install/setup.bash"

exec "$@"
