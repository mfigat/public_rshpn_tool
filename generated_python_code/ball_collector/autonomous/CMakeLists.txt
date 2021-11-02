cmake_minimum_required(VERSION 2.8.3)
project(autonomous)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	genmsg
	message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
	FILES
		MotorMessage.msg
		SensorMessage.msg
		CameraMessage.msg
		ObstacleDetected.msg
		DetectedBall.msg
)


generate_messages(DEPENDENCIES std_msgs
geometry_msgs
)

include_directories( include ${catkin_INCLUDE_DIRS} include/autonomous)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime 
)


