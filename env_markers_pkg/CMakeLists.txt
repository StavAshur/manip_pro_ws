cmake_minimum_required(VERSION 3.0.2)
project(env_markers_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  roslib
)

find_package(yaml-cpp REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_executable(sphere_marker_node src/sphere_marker_node.cpp)
target_link_libraries(sphere_marker_node
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(segment_marker_node src/segment_marker_node.cpp)
target_link_libraries(segment_marker_node ${catkin_LIBRARIES})

install(TARGETS sphere_marker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
