#!/bin/bash

# Remember to make it executable

set -e

# symbolic links
find $HOME/catkin_ws/src_extern/ -maxdepth 1 -mindepth 1 -type d -exec ln -s '{}' $HOME/catkin_ws/src/. \;

# make easy acces qgroundcontrol
touch $HOME/catkin_ws/QGroundControl.sh
chmod +x $HOME/catkin_ws/QGroundControl.sh
echo "$HOME/apps/./QGroundControl.AppImage --appimage-extract-and-run" >> $HOME/catkin_ws/QGroundControl.sh
#ln -s $HOME/apps/QGroundControl.AppImage $HOME/catkin_ws/QGroundControl

exec "$@"



