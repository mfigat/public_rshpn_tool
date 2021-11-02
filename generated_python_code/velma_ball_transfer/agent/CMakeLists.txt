cmake_minimum_required(VERSION 2.8.3)
project(agent)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	rtt_ros
	rtt_subsystem
	velma_core_cs_task_cs_msgs
	eigen_conversions
	rtt_actionlib
	common_interfaces
	subsystem_common
	controller_common
	rcprg_planner
	behavior_switch_action_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

generate_messages(DEPENDENCIES velma_core_cs_task_cs_msgs
behavior_switch_action_msgs
)

include_directories( include ${catkin_INCLUDE_DIRS} include/agent)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp rospy std_msgs 
)


