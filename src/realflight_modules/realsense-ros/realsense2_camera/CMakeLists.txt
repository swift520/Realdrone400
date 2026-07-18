cmake_minimum_required(VERSION 3.10)
project(realsense2_camera)

add_compile_options(-std=c++11)

option(BUILD_WITH_OPENMP "Use OpenMP" OFF)
option(SET_USER_BREAK_AT_STARTUP "Set user wait point in startup (for debug)" OFF)

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

# =========================
# Catkin
# =========================
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    nav_msgs
    nodelet
    image_transport
    tf
    cv_bridge
    message_generation
    ddynamic_reconfigure
    diagnostic_updater
)

# =========================
# Messages / Services
# =========================
add_message_files(
    FILES
    IMUInfo.msg
    Extrinsics.msg
    Metadata.msg
)

add_service_files(
    FILES
    DeviceInfo.srv
)

generate_messages(
    DEPENDENCIES
    sensor_msgs
    std_msgs
)

# =========================
# RealSense (Linux / Jetson)
# =========================
if(NOT WIN32)
    find_package(PkgConfig REQUIRED)
    # Link through pkg-config's imported target so headers and the resolved
    # library come from the same installation.  Using only
    # REALSENSE2_LIBRARIES ("realsense2") let the linker silently fall back to
    # an older system library when a vendor .pc file had a bad architecture
    # libdir, producing a runtime API-version mismatch.
    pkg_check_modules(REALSENSE2 REQUIRED IMPORTED_TARGET realsense2)

    message(STATUS "Using RealSense via pkg-config")
    message(STATUS "  RealSense version : ${REALSENSE2_VERSION}")
    message(STATUS "  RealSense include : ${REALSENSE2_INCLUDE_DIRS}")
    message(STATUS "  RealSense libs    : ${REALSENSE2_LINK_LIBRARIES}")
endif()

# =========================
# OpenMP (optional)
# =========================
if(BUILD_WITH_OPENMP)
    find_package(OpenMP)
    if(NOT OpenMP_FOUND)
        message(FATAL_ERROR "OpenMP requested but not found")
    endif()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if(SET_USER_BREAK_AT_STARTUP)
    add_definitions(-DBPDEBUG)
endif()

# =========================
# Build type
# =========================
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

string(TOUPPER "${CMAKE_BUILD_TYPE}" _BT)
if(_BT STREQUAL "RELEASE")
    message(STATUS "Create Release Build")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
else()
    message(STATUS "Create Debug Build")
endif()

# =========================
# Include dirs
# =========================
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# =========================
# Library
# =========================
add_library(${PROJECT_NAME}
    include/constants.h
    include/realsense_node_factory.h
    include/base_realsense_node.h
    include/t265_realsense_node.h
    src/realsense_node_factory.cpp
    src/base_realsense_node.cpp
    src/t265_realsense_node.cpp
)

add_dependencies(${PROJECT_NAME}
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${REALSENSE2_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    PkgConfig::REALSENSE2
    ${CMAKE_THREAD_LIBS_INIT}
)

# =========================
# Catkin package
# =========================
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        sensor_msgs
        std_msgs
        nodelet
        image_transport
        cv_bridge
        nav_msgs
        message_runtime
)

# =========================
# Install
# =========================
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

install(FILES nodelet_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
