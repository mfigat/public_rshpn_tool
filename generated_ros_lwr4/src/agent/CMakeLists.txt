cmake_minimum_required(VERSION 2.8.3)
project(agent)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	genmsg
)

find_package(Boost REQUIRED COMPONENTS system)

generate_messages(DEPENDENCIES std_msgs
)

include_directories( include ${catkin_INCLUDE_DIRS} include/agent)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp rospy std_msgs 
)

set( SOURCES_shared_memory
	../../../my_libs/shared_memory/shared_memory.cpp
)
set( HEADERS_shared_memory
	../../../my_libs/shared_memory/shared_memory.H
)

set( SOURCES_lwr4_kinematics_dynamics
	../../../my_libs/lwr4_kinematics_dynamics/lwr4_kinematics_dynamics.cpp
)
set( HEADERS_lwr4_kinematics_dynamics
	../../../my_libs/lwr4_kinematics_dynamics/lwr4_kinematics_dynamics.H
)

add_library( shared_memory SHARED ${SOURCES_shared_memory})
target_link_libraries( shared_memory pthread)

add_library( lwr4_kinematics_dynamics SHARED ${SOURCES_lwr4_kinematics_dynamics})
target_link_libraries( lwr4_kinematics_dynamics pthread)

add_executable(cs src/cs.cpp )
target_link_libraries(cs ${catkin_LIBRARIES} pthread rt shared_memory lwr4_kinematics_dynamics)
add_dependencies(cs agent_generate_messages_cpp)

add_executable(ve src/ve.cpp )
target_link_libraries(ve ${catkin_LIBRARIES} pthread rt shared_memory lwr4_kinematics_dynamics)
add_dependencies(ve agent_generate_messages_cpp)


