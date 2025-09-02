#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/sparkie_base/install/setup.bash"
exec "$@"
