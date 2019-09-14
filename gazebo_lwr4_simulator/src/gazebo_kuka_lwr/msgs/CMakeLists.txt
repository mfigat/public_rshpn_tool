find_package(Protobuf REQUIRED) # include the Protobuf package

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

#set (msgs # creates a list of proto messages
#  collision_map_request.proto # name of the created message
#  ${PROTOBUF_IMPORT_DIRS}/vector2d.proto
#  ${PROTOBUF_IMPORT_DIRS}/header.proto
#  ${PROTOBUF_IMPORT_DIRS}/time.proto
#)

set (msgs # creates a list of proto messages
  kuka_joints.proto # name of the created message
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs}) # build the necessary C++ headers and source files for our messages
add_library(kuka_joints_msgs SHARED ${PROTO_SRCS}) # add this message library - the same name as .proto file
target_link_libraries(kuka_joints_msgs ${PROTOBUF_LIBRARY})
