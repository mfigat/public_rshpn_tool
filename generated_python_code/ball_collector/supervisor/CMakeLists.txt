cmake_minimum_required(VERSION 2.8.3)
project(supervisor)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	genmsg
)

find_package(Boost REQUIRED COMPONENTS system)

generate_messages(DEPENDENCIES std_msgs
geometry_msgs
)

include_directories( include ${catkin_INCLUDE_DIRS} include/supervisor)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs 
)


