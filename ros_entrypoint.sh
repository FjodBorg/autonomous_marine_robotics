#!/bin/bash

# Remember to make it executable

set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
#source "$HOME/catkin_ws/devel"
exec "$@"

# link extra libraires
find $HOME/catkin_ws/src_extra -maxdepth 1 -mindepth 1 -exec ln -s ../'{}' $HOME/catkin_ws/src/ \;
