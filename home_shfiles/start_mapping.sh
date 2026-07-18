#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${WORKSPACE_DIR}/devel/setup.bash"
roslaunch fast_lio mapping_mid360.launch rviz:=false
