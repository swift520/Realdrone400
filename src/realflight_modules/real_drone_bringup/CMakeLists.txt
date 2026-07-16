cmake_minimum_required(VERSION 3.0.2)
project(real_drone_bringup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vision_pose_node src/vision_pose.cpp)
add_dependencies(vision_pose_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_pose_node ${catkin_LIBRARIES})
target_compile_options(vision_pose_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS vision_pose_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/vision_pose_watchdog.test)
endif()
