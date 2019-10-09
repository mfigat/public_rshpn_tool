cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(test_lwr4)

set(CMAKE_CXX_STANDARD 11)

set(MAIN_SOURCE
	main.cpp
)

set(SOURCES
	lwr4_kinematics_dynamics.cpp
)

set(HEADERS
	lwr4_kinematics_dynamics.h
)

set(SHM_SOURCES
	../shared_memory/shared_memory.cpp
)

set(SHM_HEADERS
	../shared_memory/shared_memory.h
)


add_library(lwr4_kinematics_dynamics SHARED ${SOURCES}) #
target_link_libraries(lwr4_kinematics_dynamics pthread)

add_library(shared_memory SHARED ${SHM_SOURCES}) #
target_link_libraries(shared_memory pthread)

###########################

# main
add_executable(test_lwr4
${MAIN_SOURCE}
${HEADERS}
${SHM_HEADERS}
)

target_compile_features(test_lwr4 PRIVATE cxx_range_for)


target_link_libraries (test_lwr4 pthread shared_memory lwr4_kinematics_dynamics rt)
install(TARGETS test_lwr4 RUNTIME DESTINATION bin)

########################
