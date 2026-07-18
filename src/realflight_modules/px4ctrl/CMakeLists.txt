cmake_minimum_required(VERSION 2.8.3)
project(px4ctrl)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  quadrotor_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  uav_utils
  mavros_msgs
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED) 

catkin_package(
  CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs quadrotor_msgs roscpp sensor_msgs std_msgs uav_utils
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
  include/${PROJECT_NAME}
  include
  src
)

add_executable(px4ctrl_node 
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
  src/localization_health.cpp
)

add_dependencies(px4ctrl_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(px4ctrl_node
  ${catkin_LIBRARIES}
)

install(TARGETS px4ctrl_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_localization_health
    test/test_localization_health.cpp
    src/localization_health.cpp
  )
  if(TARGET test_localization_health)
    target_link_libraries(test_localization_health ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_input_validity
    test/test_input_validity.cpp
    src/input.cpp
  )
  if(TARGET test_input_validity)
    target_link_libraries(test_input_validity ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_controller_safety
    test/test_controller_safety.cpp
    src/controller.cpp
    src/input.cpp
    src/PX4CtrlParam.cpp
  )
  if(TARGET test_controller_safety)
    target_link_libraries(test_controller_safety ${catkin_LIBRARIES})
  endif()
  add_rostest(test/px4ctrl_health_gate.test)
endif()

catkin_install_python(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
