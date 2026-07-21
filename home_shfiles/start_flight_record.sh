#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ ! -f "${WORKSPACE_DIR}/devel/setup.bash" ]]; then
  echo "Workspace is not built: ${WORKSPACE_DIR}/devel/setup.bash is missing." >&2
  exit 1
fi

source_workspace_setup() {
  source "${WORKSPACE_DIR}/devel/setup.bash"
}

# Source Catkin with an empty positional-parameter list. Otherwise recorder
# options such as --help are consumed by setup.bash before reaching Python.
source_workspace_setup
unset -f source_workspace_setup
set -u
export REAL_DRONE_WORKSPACE="${WORKSPACE_DIR}"

exec rosrun real_drone_bringup flight_recorder.py "$@"
