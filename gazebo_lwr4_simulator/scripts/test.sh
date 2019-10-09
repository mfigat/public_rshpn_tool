#!/bin/bash

cd ..
catkin config --extend /opt/ws_gazebo --cmake-args
catkin build

source ~/git/public_rshpn_tool/gazebo_lwr4_simulator/devel/setup.bash
export GAZEBO_MODEL_PATH=~/git/public_rshpn_tool/gazebo_lwr4_simulator/src/gazebo_kuka_lwr/gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=~/git/public_rshpn_tool/gazebo_lwr4_simulator/devel/lib:$GAZEBO_PLUGIN_PATH
gazebo -e dart
