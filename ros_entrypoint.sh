#!/bin/sh
set -e

# Basic entrypoint for ROS containers
# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep update
echo "Sourced ROS 2 ${ROS_DISTRO}"

# Source the underlay workspace, if built
cd /home/$USERNAME/thesis-mppi-model-ident
if [ -f workspace/install/setup.bash ]
then
  source workspace/install/setup.bash
  echo "Sourced Thesis base workspace"
fi

# Execute the command passed into this entrypoint
exec "$@"