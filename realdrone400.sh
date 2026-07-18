#!/bin/sh

WORKSPACE_DIR="/home/haowen2/code/REAL_DRONE_400"
REALSENSE_READY_SERVICE="/camera/stereo_module/set_parameters"

sleep 30
gnome-terminal --window -- bash -c \
"source ${WORKSPACE_DIR}/devel/setup.bash;
 # Start MAVROS only.  The vision bridge is started once, after FAST-LIO.
 roslaunch real_drone_bringup takeoff_px4.launch enable_vision_bridge:=false; exec bash" &

# Wait for the ROS master created by the first launch.  This avoids a race where
# the Realsense launch and MAVROS both try to start roscore.
if ! timeout 15 bash -c \
  "source ${WORKSPACE_DIR}/devel/setup.bash;
   until rosparam get /rosversion >/dev/null 2>&1; do sleep 0.25; done"; then
  echo "[WARN] ROS master was not ready after 15 seconds; continuing startup."
fi

# Start the Realsense driver before the recorder.  The recorder uses the
# dynamic-reconfigure service below to enable the infrared emitter.
gnome-terminal --window -- bash -c \
"source ${WORKSPACE_DIR}/devel/setup.bash;
 roslaunch realsense2_camera rs_camera.launch \
   enable_color:=true enable_depth:=true \
   enable_infra:=false enable_infra1:=true enable_infra2:=true; exec bash" &

# Give the camera time to enumerate and publish its parameter service.  Camera
# failure must not block the core flight recorder or the rest of the stack.
realsense_recorder_timeout=8
if timeout 20 bash -c \
  "source ${WORKSPACE_DIR}/devel/setup.bash;
   until rosservice info ${REALSENSE_READY_SERVICE} >/dev/null 2>&1; do sleep 0.25; done"; then
  echo "[INFO] Realsense is ready: ${REALSENSE_READY_SERVICE}"
else
  echo "[WARN] Realsense was not ready after 20 seconds; core flight recording will continue without camera topics."
  # The recorder no longer needs to repeat the full service wait in this case.
  realsense_recorder_timeout=1
fi

gnome-terminal --window -- bash -c \
"cd ${WORKSPACE_DIR};
# Start the compact recorder before sensors/mapping to capture startup transitions.
./home_shfiles/start_flight_record.sh realdrone400 --with-realsense --realsense-timeout ${realsense_recorder_timeout} --with-lidar; exec bash" &

sleep 1
gnome-terminal --window -- bash -c \
"source /home/haowen2/3rd_party/ws_livox/devel/setup.bash;
roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"  &

sleep 1
gnome-terminal --window -- bash -c \
"source ${WORKSPACE_DIR}/devel/setup.bash;
roslaunch fast_lio mapping_mid360.launch rviz:=false; exec bash"  &

sleep 1
gnome-terminal --window -- bash -c \
"source ${WORKSPACE_DIR}/devel/setup.bash;
# Start the single external-vision bridge after both FAST-LIO streams exist.
roslaunch real_drone_bringup takeoff_vrpn.launch; exec bash"  &

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
