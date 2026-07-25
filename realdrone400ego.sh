#!/usr/bin/env bash
set -e

/home/haowen2/code/REAL_DRONE_400/home_shfiles/wait_for_time_sync.sh

gnome-terminal --window -- bash -c \
"source /home/haowen2/code/REAL_DRONE_400/devel/setup.bash;
roslaunch real_drone_bringup takeoff_px4.launch enable_vision_bridge:=false; exec bash"  &

/home/haowen2/code/REAL_DRONE_400/home_shfiles/start_realsense_recording.sh realdrone400 --with-lidar

gnome-terminal --window -- bash -c \
"source /home/haowen2/3rd_party/ws_livox/devel/setup.bash;
roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"  &

sleep 1
gnome-terminal --window -- bash -c \
"source /home/haowen2/code/REAL_DRONE_400/devel/setup.bash;
roslaunch fast_lio mapping_mid360.launch rviz:=false; exec bash"  &

sleep 1
gnome-terminal --window -- bash -c \
"source /home/haowen2/code/REAL_DRONE_400/devel/setup.bash;
roslaunch real_drone_bringup takeoff_vrpn.launch; exec bash"  &

sleep 1
gnome-terminal --window -- bash -c \
"source /home/haowen2/code/REAL_DRONE_400/devel/setup.bash;
roslaunch px4ctrl run_ctrl.launch; exec bash"  &

sleep 1
gnome-terminal --window -- bash -c \
"source /home/haowen2/code/REAL_DRONE_400/devel/setup.bash;
rostopic echo /debugPx4ctrl; exec bash"  &

sleep 3
gnome-terminal --window -- bash -c \
"source /home/haowen2/code/REAL_DRONE_400/devel/setup.bash;
rostopic pub -1  /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand 'takeoff_land_cmd: 1'; exec bash"  &

sleep 1
gnome-terminal --window -- bash -c \
"source /home/haowen2/code/REAL_DRONE_400/devel/setup.bash;
roslaunch ego_planner single_run_in_exp.launch; exec bash"  &

sleep 1
gnome-terminal --window -- bash -c \
"source /home/haowen2/code/REAL_DRONE_400/devel/setup.bash;
roslaunch ego_planner rviz.launch; exec bash"

sleep 20
gnome-terminal --window -- bash -c \
"source /home/haowen2/code/REAL_DRONE_400/devel/setup.bash;
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'; exec bash"

sleep 40
gnome-terminal --window -- bash -c \
"source /home/haowen2/code/REAL_DRONE_400/devel/setup.bash;
rostopic pub -1  /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand 'takeoff_land_cmd: 2'; exec bash"  &
