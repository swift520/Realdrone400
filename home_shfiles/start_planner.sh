#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${WORKSPACE_DIR}/devel/setup.bash"
# roslaunch realsense2_camera rs_camera.launch & sleep 3;
roslaunch ego_planner single_run_in_exp.launch
