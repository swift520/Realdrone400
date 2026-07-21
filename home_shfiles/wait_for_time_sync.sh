#!/usr/bin/env bash
set -euo pipefail

SYNC_MARKER="/run/systemd/timesync/synchronized"
SYNC_TIMEOUT_SECONDS="${REAL_DRONE_TIME_SYNC_TIMEOUT:-60}"

if [[ ! "${SYNC_TIMEOUT_SECONDS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "[ERROR] REAL_DRONE_TIME_SYNC_TIMEOUT must be a positive integer." >&2
  exit 2
fi

time_is_synchronized() {
  [[ "$(timedatectl show --property=NTPSynchronized --value 2>/dev/null || true)" == "yes" ]] || \
    [[ -e "${SYNC_MARKER}" ]]
}

echo "[INFO] Waiting for system time synchronization..."
wait_started_at=${SECONDS}
while ! time_is_synchronized; do
  if (( SECONDS - wait_started_at >= SYNC_TIMEOUT_SECONDS )); then
    echo "[WARN] System time did not synchronize within ${SYNC_TIMEOUT_SECONDS}s; continuing with potentially inaccurate log timestamps." >&2
    exit 0
  fi
  sleep 1
done

python3 - <<'PY'
import time

stable_seconds = 3.0
maximum_step = 0.2
reference_offset = time.time() - time.monotonic()
stable_since = time.monotonic()

while time.monotonic() - stable_since < stable_seconds:
    time.sleep(0.1)
    current_offset = time.time() - time.monotonic()
    if abs(current_offset - reference_offset) > maximum_step:
        reference_offset = current_offset
        stable_since = time.monotonic()
PY

echo "[INFO] System time is synchronized and stable."
