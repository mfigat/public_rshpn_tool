cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)


project(gazebo_kuka_lwr)

FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED ) 			# dodane z tutoriala

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	#  roscpp
	#  geometry_msgs
	#  visualization_msgs
	#  urdf
	roscpp REQUIRED
	rospy
	std_msgs REQUIRED
	genmsg
	cmake_modules
)


find_package(GAZEBO REQUIRED)
#find_package(orocos_kdl REQUIRED)
#find_package(kdl_parser REQUIRED)

set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11") # dodane z tutoriala

include (FindPkgConfig) # dodane z tutoriala
if (PKG_CONFIG_FOUND)   # dodane z tutoriala
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()


# Export package information (replaces catkin_package() macro)
catkin_package(
  LIBRARIES kuka_lwr_plugin
	CATKIN_DEPENDS roscpp rospy std_msgs 
)



include_directories(
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
	${SDF_INCLUDE_DIRS}							# dodane z tutoriala
	${CMAKE_CURRENT_BINARY_DIR}/msgs # include directories
#	lwr4_kinematics_dynamics 	# moje biblioteki - wykomentowalem
#	shared_memory 	# moje biblioteki
)


set( SOURCES_shared_memory
	../../../ros/my_libs/shared_memory/shared_memory.cpp
)
set( HEADERS_shared_memory
	../../../ros/my_libs/shared_memory/shared_memory.H
)

set( SOURCES_lwr4_kinematics_dynamics
	../../../ros/my_libs/lwr4_kinematics_dynamics/lwr4_kinematics_dynamics.cpp
)
set( HEADERS_lwr4_kinematics_dynamics
	../../../ros/my_libs/lwr4_kinematics_dynamics/lwr4_kinematics_dynamics.H
)




include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

link_directories(${catkin_INCLUDE_DIRS}  ${CMAKE_CURRENT_BINARY_DIR}/msgs) 	# dodane z tutoriala
add_subdirectory(msgs) # tells CMake to look in it for a CMakeLists.txt file 	# dodane z tutoriala


add_library( shared_memory SHARED ${SOURCES_shared_memory})
target_link_libraries( shared_memory pthread)

add_library( lwr4_kinematics_dynamics SHARED ${SOURCES_lwr4_kinematics_dynamics})
target_link_libraries( lwr4_kinematics_dynamics pthread)


# Test dodania moich bibliotek
# lwr4+ biblioteka
#link_directories(${catkin_INCLUDE_DIRS}  ${CMAKE_CURRENT_BINARY_DIR}/lwr4_kinematics_dynamics) 	#
#add_subdirectory(lwr4_kinematics_dynamics) # tells CMake to look in it for a CMakeLists.txt file 	#

# share_memory - biblioteka
#link_directories(${catkin_INCLUDE_DIRS}  ${CMAKE_CURRENT_BINARY_DIR}/shared_memory) 	#
#add_subdirectory(shared_memory) # tells CMake to look in it for a CMakeLists.txt file 	#


# BEGIN - KUKA
add_library(
	kuka_lwr_plugin
	src/kuka_lwr_plugin.cpp
)

target_link_libraries(
	kuka_lwr_plugin
	kuka_joints_msgs 	# biblioteka do komunikacji
	lwr4_kinematics_dynamics # moja biblioteka do sterowania manipulatorem lwr4+
	shared_memory		 # moja biblioteka do komunikacji za pomocą pamięci współdzielonej
	${SDF_LIBRARIES}	# dodane - wiadomości
	${Boost_LIBRARIES}	# dodane - wiadomości
	${catkin_LIBRARIES}
	${GAZEBO_LIBRARIES}
	${roscpp_LIBRARIES}
)




add_dependencies(
	kuka_lwr_plugin
	kuka_joints_msgs
	lwr4_kinematics_dynamics 		# dodać nazwę biblioteki
	shared_memory 				# dodać nazwę biblioteki
) # tells to build first the kuka_joints_msgs

# END - KUKA

install(TARGETS kuka_lwr_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
