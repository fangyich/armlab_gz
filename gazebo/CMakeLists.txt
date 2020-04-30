cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)
find_package(Boost 1.65 COMPONENTS system date_time regex REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include_directories(
	${GAZEBO_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR}/msgs
	${CMAKE_CURRENT_SOURCE_DIR}/msgs
)
link_directories(
	${GAZEBO_LIBRARY_DIRS}
	${CMAKE_CURRENT_BINARY_DIR}/msgs
)

# Build messages
add_subdirectory(msgs)

# Build client
add_executable(c_client c_client.cpp)
target_link_libraries(c_client rexarm_poses_msgs cube_info_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(c_client rexarm_poses_msgs cube_info_msgs)

# Build plugins
add_library(rexarm_plugin SHARED rexarm_plugin.cc)
target_link_libraries(rexarm_plugin rexarm_poses_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(rexarm_plugin rexarm_poses_msgs)

add_library(trjmarker_plugin SHARED trjmarker_plugin.cc)
target_link_libraries(trjmarker_plugin rexarm_poses_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(cubefactory_plugin SHARED cubefactory_plugin.cc)
target_link_libraries(cubefactory_plugin cube_info_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
