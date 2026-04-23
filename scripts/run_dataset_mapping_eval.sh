#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

VENV_DIR="${VENV_DIR:-${WORKSPACE_DIR}/.venv_ros_ml}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-226}"
BAG_RATE="${BAG_RATE:-1.0}"
STARTUP_WAIT="${STARTUP_WAIT:-12}"
EVAL_TIMEOUT="${EVAL_TIMEOUT:-90}"
IDLE_TIMEOUT="${IDLE_TIMEOUT:-8}"
TRACK="${TRACK:-acceleration}"
RVIZ="${RVIZ:-false}"
DUPLICATE_THRESHOLD="${DUPLICATE_THRESHOLD:-0.75}"
LIDAR_BACKEND="${LIDAR_BACKEND:-pointpillars}"
DRD26_PATH_CONFIG="${DRD26_PATH_CONFIG:-${WORKSPACE_DIR}/LocalizationMapping/config/hardcoded_paths.ini}"

source "${WORKSPACE_DIR}/scripts/activate_ros_ml.sh" >/tmp/drd26_activate_ros_ml.log
set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

DATASET_DIR="${DATASET_DIR:-$(
python3 - "${DRD26_PATH_CONFIG}" <<'PY'
from configparser import ConfigParser
from pathlib import Path
import sys

config = Path(sys.argv[1])
parser = ConfigParser()
parser.read(config, encoding="utf-8")
print(parser.get("datasets", "rosbag_2026_02_05"))
PY
)}"

if [[ ! -d "${DATASET_DIR}" ]]; then
  echo "Dataset directory not found: ${DATASET_DIR}" >&2
  exit 1
fi

if [[ "${LIDAR_BACKEND}" == "pointpillars" ]] && \
   ! ros2 pkg executables trt_cone_detector | awk '{print $2}' | grep -qx "trt_infer_node"; then
  echo "PointPillars backend selected, but trt_infer_node is not installed." >&2
  echo "Build on a machine with CUDA/TensorRT, or run with LIDAR_BACKEND=cluster for the legacy detector." >&2
  exit 1
fi

export ROS_DOMAIN_ID
export PYTHONUNBUFFERED=1
export DRD26_PATH_CONFIG

LOG_DIR="${LOG_DIR:-${WORKSPACE_DIR}/log/eval/dataset_mapping_eval_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${LOG_DIR}"

cleanup() {
  set +e
  for pid in ${BAG_PID:-} ${MONITOR_PID:-} ${STACK_PID:-}; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  sleep 2
  for pid in ${BAG_PID:-} ${MONITOR_PID:-} ${STACK_PID:-}; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

echo "Workspace: ${WORKSPACE_DIR}"
echo "Dataset: ${DATASET_DIR}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "Bag rate: ${BAG_RATE}"
echo "Track: ${TRACK}"
echo "LiDAR backend: ${LIDAR_BACKEND}"
echo "Eval timeout: ${EVAL_TIMEOUT}"
echo "Idle timeout: ${IDLE_TIMEOUT}"
echo "Duplicate threshold: ${DUPLICATE_THRESHOLD}"
echo "Logs: ${LOG_DIR}"

ros2 launch racingbrain localization_mapping.launch.py \
  "track:=${TRACK}" \
  "rviz:=${RVIZ}" \
  "lidar_backend:=${LIDAR_BACKEND}" \
  "eval_debug:=true" \
  "enable_planning:=false" \
  "enable_health:=true" \
  >"${LOG_DIR}/stack.log" 2>&1 &
STACK_PID=$!

sleep "${STARTUP_WAIT}"

python3 "${WORKSPACE_DIR}/scripts/eval/drd26_eval_monitor.py" \
  --log-dir "${LOG_DIR}" \
  --dataset "${DATASET_DIR}" \
  --timeout "${EVAL_TIMEOUT}" \
  --idle-timeout "${IDLE_TIMEOUT}" \
  --duplicate-threshold "${DUPLICATE_THRESHOLD}" \
  >"${LOG_DIR}/eval_monitor.log" 2>&1 &
MONITOR_PID=$!

sleep 1

ros2 bag play "${DATASET_DIR}" \
  --rate "${BAG_RATE}" \
  --topics /camera1/image_raw /lidar_points /gongji_gnss_ins_64 \
  >"${LOG_DIR}/bag.log" 2>&1 &
BAG_PID=$!

set +e
wait "${MONITOR_PID}"
MONITOR_STATUS=$?
set -e

echo
echo "--- eval monitor tail ---"
tail -n 80 "${LOG_DIR}/eval_monitor.log" || true
echo
echo "--- stack tail ---"
tail -n 80 "${LOG_DIR}/stack.log" || true
echo
echo "--- bag tail ---"
tail -n 30 "${LOG_DIR}/bag.log" || true
echo
echo "Summary: ${LOG_DIR}/summary.json"
echo "Report: ${LOG_DIR}/report.md"

exit "${MONITOR_STATUS}"
