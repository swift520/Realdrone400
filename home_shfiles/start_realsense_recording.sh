#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_FILE="${WORKSPACE_DIR}/devel/setup.bash"
CAMERA_SERVICE="/camera/stereo_module/set_parameters"

source_workspace_setup() {
  source "${SETUP_FILE}"
}

# Keep this script's recorder arguments away from Catkin's setup parser.
source_workspace_setup
unset -f source_workspace_setup
set -u

wait_for_master() {
  timeout 15 bash -c \
    'until rosparam get /rosversion >/dev/null 2>&1; do sleep 0.25; done'
}

wait_for_service() {
  timeout 20 bash -c \
    'until rosservice info "$1" >/dev/null 2>&1; do sleep 0.25; done' \
    bash "$1"
}

wait_for_recorder() {
  timeout 30 bash -c \
    'until rosnode info /flight_bag >/dev/null 2>&1; do sleep 0.25; done'
}

if ! wait_for_master; then
  echo "[WARN] ROS master was not ready after 15 seconds; continuing startup."
fi

if ! rosservice info "${CAMERA_SERVICE}" >/dev/null 2>&1; then
  gnome-terminal --window -- bash -c '
    setup_file="$1"
    source_workspace_setup() {
      source "$setup_file"
    }
    source_workspace_setup
    unset -f source_workspace_setup
    roslaunch --wait realsense2_camera rs_camera.launch
    exec bash
  ' bash "${SETUP_FILE}" &
fi

recorder_args=("$@" --with-realsense)
if ! wait_for_service "${CAMERA_SERVICE}"; then
  echo "[WARN] Realsense unavailable; recording will continue without camera topics."
  recorder_args+=(--realsense-timeout 1)
fi

gnome-terminal --window -- bash -c '"$@"; exec bash' bash \
  "${SCRIPT_DIR}/start_flight_record.sh" "${recorder_args[@]}" &

if ! wait_for_recorder; then
  echo "[WARN] Flight recorder was not ready after 30 seconds; continuing startup."
fi
