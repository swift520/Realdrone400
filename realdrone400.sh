#!/usr/bin/env bash
set -e

WORKSPACE_DIR="/home/haowen2/code/REAL_DRONE_400"
REAL_DRONE_SETUP="${WORKSPACE_DIR}/devel/setup.bash"
LIVOX_SETUP="/home/haowen2/3rd_party/ws_livox/devel/setup.bash"

start_terminal() {
  local setup_file="$1"
  shift
  gnome-terminal --window -- bash -c '
    setup_file="$1"
    shift
    source_workspace_setup() {
      source "$setup_file"
    }
    source_workspace_setup
    unset -f source_workspace_setup
    "$@"
    exec bash
  ' bash "${setup_file}" "$@" &
}

"${WORKSPACE_DIR}/home_shfiles/wait_for_time_sync.sh"

start_terminal "${REAL_DRONE_SETUP}" \
  roslaunch real_drone_bringup takeoff_px4.launch enable_vision_bridge:=false

"${WORKSPACE_DIR}/home_shfiles/start_realsense_recording.sh" realdrone400 --with-lidar

start_terminal "${LIVOX_SETUP}" roslaunch livox_ros_driver2 msg_MID360.launch

sleep 1
start_terminal "${REAL_DRONE_SETUP}" roslaunch fast_lio mapping_mid360.launch rviz:=false

sleep 1
start_terminal "${REAL_DRONE_SETUP}" roslaunch real_drone_bringup takeoff_vrpn.launch


#sleep 1
#gnome-terminal --window -- bash -c \
#"source /opt/ros/noetic/setup.bash;
#bash record_livox.sh; exec bash"  &

#sleep 1
#gnome-terminal --window -- bash -c \
#"source /home/haowen2/code/REAL_DRONE_400/devel/setup.bash;
#roslaunch px4ctrl run_ctrl.launch; exec bash"  &

#sleep 1
#gnome-terminal --window -- bash -c \
#"source /home/haowen2/code/REAL_DRONE_400/devel/setup.bash;
#rostopic echo /debugPx4ctrl; exec bash"  &

#sleep 1
#gnome-terminal --window -- bash -c \
#"source /home/haowen2/code/REAL_DRONE_400/devel/setup.bash;
#rostopic pub -1  /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"; exec bash"  &

#sleep 1
#gnome-terminal --window -- bash -c \
#"source /home/haowen2/code/REAL_DRONE_400/devel/setup.bash;
#rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2"; exec bash"  &

#sleep 1
#gnome-terminal --window -- bash -c \
#"source /home/haowen2/code/REAL_DRONE_400/devel/setup.bash;
#roslaunch ego_planner single_run_in_exp.launch; exec bash"  &

#sleep 1
#gnome-terminal --window -- bash -c \
#"source /home/haowen2/code/REAL_DRONE_400/devel/setup.bash;
#roslaunch ego_planner rviz.launch; exec bash"
