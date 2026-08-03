#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_FILE="${WORKSPACE_DIR}/devel/setup.bash"
CAMERA_SERVICE="/camera/stereo_module/set_parameters"
CAMERA_PARAM_NAMESPACE="/camera/realsense2_camera"
# Reduce load at the camera source while preserving the original raw image
# topics and keeping the stereo-module streams on one supported frame rate.
CAMERA_FPS=15

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

verify_camera_fps() {
  local stream actual_fps
  for stream in color depth infra; do
    if ! actual_fps="$(rosparam get "${CAMERA_PARAM_NAMESPACE}/${stream}_fps" 2>/dev/null)"; then
      echo "[ERROR] Cannot read ${CAMERA_PARAM_NAMESPACE}/${stream}_fps." >&2
      return 1
    fi
    if [[ "${actual_fps}" != "${CAMERA_FPS}" ]]; then
      echo "[ERROR] Realsense ${stream} stream is ${actual_fps} Hz; ${CAMERA_FPS} Hz is required." >&2
      return 1
    fi
  done
}

verify_topic_rate() {
  local topic="$1" samples measured_fps
  if ! samples="$(timeout 5 rostopic echo -p -n 12 "${topic}/header/stamp" 2>/dev/null)"; then
    echo "[ERROR] Cannot collect enough timestamped samples from ${topic}." >&2
    return 1
  fi
  measured_fps="$(awk -F, '
    NR == 2 { first = $2 }
    NR > 1 { last = $2; count += 1 }
    END {
      if (count > 1 && last > first) {
        printf "%.2f", (count - 1) * 1000000000.0 / (last - first)
      }
    }
  ' <<<"${samples}")"
  if [[ -z "${measured_fps}" ]] || ! awk \
      -v actual="${measured_fps}" -v expected="${CAMERA_FPS}" \
      'BEGIN { exit !(actual >= expected * 0.75 && actual <= expected * 1.25) }'; then
    echo "[ERROR] ${topic} measured ${measured_fps:-unknown} Hz; expected approximately ${CAMERA_FPS} Hz." >&2
    return 1
  fi
  echo "[INFO] ${topic} measured ${measured_fps} Hz."
}

verify_camera_rates() {
  verify_topic_rate "/camera/color/image_raw" &&
    verify_topic_rate "/camera/depth/image_rect_raw"
}

if ! wait_for_master; then
  echo "[WARN] ROS master was not ready after 15 seconds; continuing startup."
fi

if ! rosservice info "${CAMERA_SERVICE}" >/dev/null 2>&1; then
  gnome-terminal --window -- bash -c '
    setup_file="$1"
    camera_fps="$2"
    source_workspace_setup() {
      source "$setup_file"
    }
    source_workspace_setup
    unset -f source_workspace_setup
    roslaunch --wait realsense2_camera rs_camera.launch \
      depth_fps:="$camera_fps" infra_fps:="$camera_fps" color_fps:="$camera_fps"
    exec bash
  ' bash "${SETUP_FILE}" "${CAMERA_FPS}" &
fi

recorder_args=("$@" --with-realsense)
if ! wait_for_service "${CAMERA_SERVICE}"; then
  echo "[WARN] Realsense unavailable; recording will continue without camera topics."
  recorder_args+=(--realsense-timeout 1)
elif ! verify_camera_fps || ! verify_camera_rates; then
  echo "[ERROR] Stop the Realsense node and rerun this script so it can start all image streams at ${CAMERA_FPS} Hz." >&2
  exit 1
else
  echo "[INFO] Realsense raw RGB-D is configured at ${CAMERA_FPS} Hz."
fi

gnome-terminal --window -- bash -c '"$@"; exec bash' bash \
  "${SCRIPT_DIR}/start_flight_record.sh" "${recorder_args[@]}" &

if ! wait_for_recorder; then
  echo "[WARN] Flight recorder was not ready after 30 seconds; continuing startup."
fi
