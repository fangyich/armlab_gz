cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set(rexarm_msgs
    rexarm_poses.proto
    #${PROTOBUF_IMPORT_DIRS}/pose.proto
    #${PROTOBUF_IMPORT_DIRS}/vector3d.proto
    #${PROTOBUF_IMPORT_DIRS}/quaternion.proto
)

set(cube_msgs
    cube_info.proto
)


PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${rexarm_msgs})
#set(PROTO_SRCS "${PROTO_SRCS} ${MSG_SRCS}")
add_library(rexarm_poses_msgs SHARED ${PROTO_SRCS})
target_link_libraries(rexarm_poses_msgs ${PROTOBUF_LIBRARY})

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${cube_msgs})
#set(PROTO_SRCS "${PROTO_SRCS} ${MSG_SRCS}")
add_library(cube_info_msgs SHARED ${PROTO_SRCS})
target_link_libraries(cube_info_msgs ${PROTOBUF_LIBRARY})


