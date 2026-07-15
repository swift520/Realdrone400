#!/bin/bash

# 1. 启动 MAVROS 飞控链路
source /home/haowen2/code/REAL_DRONE_400/devel/setup.bash
roslaunch real_drone_bringup takeoff_px4.launch & sleep 5

# 2. 启动激光雷达
source /home/haowen2/3rd_party/ws_livox/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
