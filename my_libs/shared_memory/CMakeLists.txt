cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(test_shared_memory)

set(MAIN_SOURCE
	main.cpp
	producer.cpp
	consumer.cpp
)

set(PRODUCER_SOURCE
	producer.cpp
)

set(CONSUMER_SOURCE
	consumer.cpp
)

set(SOURCES
	shared_memory.cpp
)

set(HEADERS
	shared_memory.h
)


add_library(shared_memory SHARED ${SOURCES}) # 
target_link_libraries(shared_memory pthread)

###########################

# producer
add_executable(producer
${PRODUCER_SOURCE}
${HEADERS}
)
target_link_libraries (producer pthread shared_memory rt )
install(TARGETS producer RUNTIME DESTINATION bin)

######################

# consumer
add_executable(consumer
${CONSUMER_SOURCE}
${HEADERS}
)
target_link_libraries (consumer pthread shared_memory rt)
install(TARGETS consumer RUNTIME DESTINATION bin)

########################
