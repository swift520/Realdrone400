#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ ! -f "${WORKSPACE_DIR}/devel/setup.bash" ]]; then
  echo "Workspace is not built: ${WORKSPACE_DIR}/devel/setup.bash is missing." >&2
  exit 1
fi

source "${WORKSPACE_DIR}/devel/setup.bash"
export REAL_DRONE_WORKSPACE="${WORKSPACE_DIR}"

exec rosrun real_drone_bringup flight_recorder.py "$@"
