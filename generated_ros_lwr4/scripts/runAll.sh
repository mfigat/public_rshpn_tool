#!/bin/bash
cd ~/git/public_rshpn_tool/generated_ros_lwr4/
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
roslaunch agent agent_verbose.launch &
