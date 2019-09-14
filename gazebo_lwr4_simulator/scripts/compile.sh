#!/bin/bash

cd ..

source /opt/ros/melodic/setup.bash
catkin config --extend /opt/ws_gazebo --cmake-args
catkin build
