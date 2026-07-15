cmake_minimum_required(VERSION 3.0.2)
project(real_drone_bringup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf
  tf2
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp tf tf2 tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vision_pose_node src/vision_pose.cpp)
add_dependencies(vision_pose_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_pose_node ${catkin_LIBRARIES})

install(TARGETS vision_pose_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
