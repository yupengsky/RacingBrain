#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-226}"
BAG_RATE="${BAG_RATE:-1.0}"
IDLE_TIMEOUT="${IDLE_TIMEOUT:-12}"
STARTUP_TIMEOUT="${STARTUP_TIMEOUT:-60}"
FPS="${FPS:-10}"
OUTPUT_DIR="${OUTPUT_DIR:-${HOME}/Videos}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_PATH="${OUTPUT_PATH:-${OUTPUT_DIR}/racingbrain_mapping_demo_${STAMP}.mp4}"
LOG_DIR="$(mktemp -d "/tmp/racingbrain_mapping_demo_${STAMP}_XXXX")"

mkdir -p "${OUTPUT_DIR}"

cleanup() {
  set +e
  if [[ -n "${CHAIN_PID:-}" ]] && kill -0 "${CHAIN_PID}" 2>/dev/null; then
    kill -INT "${CHAIN_PID}" 2>/dev/null || true
  fi
  if [[ -n "${RECORDER_PID:-}" ]] && kill -0 "${RECORDER_PID}" 2>/dev/null; then
    kill -INT "${RECORDER_PID}" 2>/dev/null || true
  fi
  sleep 2
  if [[ -n "${CHAIN_PID:-}" ]] && kill -0 "${CHAIN_PID}" 2>/dev/null; then
    kill -TERM "${CHAIN_PID}" 2>/dev/null || true
  fi
  if [[ -n "${RECORDER_PID:-}" ]] && kill -0 "${RECORDER_PID}" 2>/dev/null; then
    kill -TERM "${RECORDER_PID}" 2>/dev/null || true
  fi
  rm -rf "${LOG_DIR}"
}
trap cleanup EXIT

echo "Workspace: ${WORKSPACE_DIR}"
echo "Video: ${OUTPUT_PATH}"
echo "Bag rate: ${BAG_RATE}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"

export ROS_DOMAIN_ID

source "${WORKSPACE_DIR}/scripts/activate_ros_ml.sh" >/tmp/drd26_activate_ros_ml.log
set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

"${WORKSPACE_DIR}/.venv_ros_ml/bin/python" \
  "${WORKSPACE_DIR}/scripts/record_mapping_demo.py" \
  --output "${OUTPUT_PATH}" \
  --fps "${FPS}" \
  --idle-timeout "${IDLE_TIMEOUT}" \
  --startup-timeout "${STARTUP_TIMEOUT}" \
  >"${LOG_DIR}/recorder.log" 2>&1 &
RECORDER_PID=$!

sleep 2

LOG_DIR="${LOG_DIR}" \
  BAG_RATE="${BAG_RATE}" \
  RVIZ=true \
  KEEP_RUNNING=true \
  "${WORKSPACE_DIR}/scripts/run_dataset_mapping_chain.sh" \
  >"${LOG_DIR}/chain.log" 2>&1 &
CHAIN_PID=$!

wait "${CHAIN_PID}"
CHAIN_STATUS=$?

wait "${RECORDER_PID}"
RECORDER_STATUS=$?

echo "Video saved to: ${OUTPUT_PATH}"

if [[ "${CHAIN_STATUS}" -ne 0 ]]; then
  echo "Chain failed. Temporary logs kept at: ${LOG_DIR}" >&2
  trap - EXIT
  exit "${CHAIN_STATUS}"
fi

if [[ "${RECORDER_STATUS}" -ne 0 ]]; then
  echo "Recorder failed. Temporary logs kept at: ${LOG_DIR}" >&2
  trap - EXIT
  exit "${RECORDER_STATUS}"
fi

trap - EXIT
cleanup
