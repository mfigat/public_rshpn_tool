#!/bin/bash

cd ..
catkin config --extend /opt/ws_gazebo --cmake-args
catkin build

source /home/maximus/git/projekty/ws_preludium/devel/setup.bash
export GAZEBO_MODEL_PATH=/home/maximus/git/projekty/ws_preludium/src/gazebo_kuka_lwr/gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=/home/maximus/git/projekty/ws_preludium/devel/lib:$GAZEBO_PLUGIN_PATH
gazebo -e dart
