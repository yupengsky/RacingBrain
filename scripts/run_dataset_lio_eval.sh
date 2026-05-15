#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-227}"
BAG_RATE="${BAG_RATE:-0.5}"
STARTUP_WAIT="${STARTUP_WAIT:-8}"
SHUTDOWN_WAIT="${SHUTDOWN_WAIT:-3}"
DATASET_CONFIG_KEY="${DATASET_CONFIG_KEY:-rosbag_2026_05_10_lio_lidar_downsample}"
DRD26_PATH_CONFIG="${DRD26_PATH_CONFIG:-${WORKSPACE_DIR}/LocalizationMapping/config/hardcoded_paths.ini}"
LOG_DIR="${LOG_DIR:-${WORKSPACE_DIR}/log/runtime/dataset_lio_eval_$(date +%Y%m%d_%H%M%S)}"
OUTPUT_DIR="${OUTPUT_DIR:-${LOG_DIR}/metrics}"
LIO_PARAMS_FILE="${LIO_PARAMS_FILE:-${WORKSPACE_DIR}/LocalizationMapping/RacingBrain/config/lio_sam_racingbrain.yaml}"
LIO_SAM_SETUP="${LIO_SAM_SETUP:-${WORKSPACE_DIR}/.ros_deps/lio_sam_ws/install/setup.bash}"
INPUT_CLOUD_TOPIC="${INPUT_CLOUD_TOPIC:-/lidar_points}"
ADAPTED_CLOUD_TOPIC="${ADAPTED_CLOUD_TOPIC:-/points}"
GNSS_TOPIC="${GNSS_TOPIC:-/gongji_gnss_ins_64}"
INPUT_IMU_TOPIC="${INPUT_IMU_TOPIC:-/imu}"
IMU_TOPIC="${IMU_TOPIC:-/imu_lio}"
LIO_ODOM_TOPIC="${LIO_ODOM_TOPIC:-/lio_sam/mapping/odometry}"
LIDAR_FRAME="${LIDAR_FRAME:-base_link}"
N_SCAN="${N_SCAN:-64}"
SCAN_PERIOD_SEC="${SCAN_PERIOD_SEC:-0.1}"
BAG_TOPICS="${BAG_TOPICS:-${INPUT_CLOUD_TOPIC} ${GNSS_TOPIC} ${INPUT_IMU_TOPIC}}"

source "${WORKSPACE_DIR}/scripts/activate_ros_ml.sh" >/tmp/racingbrain_activate_ros_ml.log
set +u
if [[ -f "${LIO_SAM_SETUP}" ]]; then
  source "${LIO_SAM_SETUP}"
fi
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

DATASET_DIR="${DATASET_DIR:-$(
python3 - "${DRD26_PATH_CONFIG}" "${DATASET_CONFIG_KEY}" <<'PY'
from configparser import ConfigParser
from pathlib import Path
import sys

config = Path(sys.argv[1])
key = sys.argv[2]
parser = ConfigParser()
parser.read(config, encoding="utf-8")
print(parser.get("datasets", key))
PY
)}"

if [[ ! -d "${DATASET_DIR}" ]]; then
  echo "Dataset directory not found: ${DATASET_DIR}" >&2
  exit 1
fi

if ! ros2 pkg prefix lio_sam >/dev/null 2>&1; then
  echo "ROS 2 package 'lio_sam' is not available in the sourced workspace." >&2
  echo "Clone https://github.com/TixiaoShan/LIO-SAM, checkout ros2, build it, then source install/setup.bash." >&2
  exit 1
fi

mkdir -p "${LOG_DIR}" "${OUTPUT_DIR}"
read -r -a BAG_TOPIC_ARGS <<< "${BAG_TOPICS}"

export ROS_DOMAIN_ID
export PYTHONUNBUFFERED=1

cleanup() {
  set +e
  for pid in ${BAG_PID:-} ${EVAL_PID:-}; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  sleep "${SHUTDOWN_WAIT}"
  for pid in ${BAG_PID:-} ${EVAL_PID:-}; do
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
echo "Bag topics: ${BAG_TOPICS}"
echo "LIO params: ${LIO_PARAMS_FILE}"
echo "LIO setup: ${LIO_SAM_SETUP}"
echo "Raw IMU topic: ${INPUT_IMU_TOPIC}"
echo "LIO IMU topic: ${IMU_TOPIC}"
echo "Metrics: ${OUTPUT_DIR}"
echo "Logs: ${LOG_DIR}"

ros2 launch racingbrain lio_dataset_eval.launch.py \
  "run_lio_sam:=true" \
  "run_pointcloud_adapter:=true" \
  "run_error_eval:=true" \
  "input_cloud_topic:=${INPUT_CLOUD_TOPIC}" \
  "adapted_cloud_topic:=${ADAPTED_CLOUD_TOPIC}" \
  "gnss_topic:=${GNSS_TOPIC}" \
  "input_imu_topic:=${INPUT_IMU_TOPIC}" \
  "imu_topic:=${IMU_TOPIC}" \
  "lio_odom_topic:=${LIO_ODOM_TOPIC}" \
  "output_dir:=${OUTPUT_DIR}" \
  "lio_params_file:=${LIO_PARAMS_FILE}" \
  "lidar_frame:=${LIDAR_FRAME}" \
  "n_scan:=${N_SCAN}" \
  "scan_period_sec:=${SCAN_PERIOD_SEC}" \
  >"${LOG_DIR}/lio_eval_stack.log" 2>&1 &
EVAL_PID=$!

sleep "${STARTUP_WAIT}"

ros2 bag play "${DATASET_DIR}" \
  --rate "${BAG_RATE}" \
  --topics "${BAG_TOPIC_ARGS[@]}" \
  >"${LOG_DIR}/bag.log" 2>&1 &
BAG_PID=$!

wait "${BAG_PID}"
sleep "${SHUTDOWN_WAIT}"
cleanup

if [[ -f "${OUTPUT_DIR}/summary.json" ]]; then
  cat "${OUTPUT_DIR}/summary.json"
else
  echo "No summary.json was produced. Check ${LOG_DIR}/lio_eval_stack.log" >&2
  exit 1
fi
