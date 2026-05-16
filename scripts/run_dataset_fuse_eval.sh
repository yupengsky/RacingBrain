#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

DATASET_DIR="${DATASET_DIR:-/media/yupeng/Ventoy/rosbag2_2026_05_10-15_02_50}"
RUN_NAME="${RUN_NAME:-fuse_eval_2026_05_10-15_02_50}"
OUTPUT_DIR="${OUTPUT_DIR:-${WORKSPACE_DIR}/results/${RUN_NAME}}"
OUTPUT_HTML="${OUTPUT_HTML:-${WORKSPACE_DIR}/results/${RUN_NAME}.html}"
OUTAGES="${OUTAGES:-45:8,185:10,360:8}"
SCENARIO_MODE="${SCENARIO_MODE:-severity_grid}"
GAP_COUNT="${GAP_COUNT:-}"
GAP_DURATION_SEC="${GAP_DURATION_SEC:-}"
DURATION_SEC="${DURATION_SEC:-0}"
START_SEC="${START_SEC:-0}"
MAX_CLOUDS="${MAX_CLOUDS:-0}"
CLOUD_STRIDE="${CLOUD_STRIDE:-1}"
IMU_GYRO_SCALE="${IMU_GYRO_SCALE:-0.04348764102608839}"

set +u
if [[ -f "${WORKSPACE_DIR}/.venv_ros_ml/bin/activate" ]]; then
  source "${WORKSPACE_DIR}/scripts/activate_ros_ml.sh" >/tmp/racingbrain_activate_ros_ml.log
else
  source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
fi
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

mkdir -p "${OUTPUT_DIR}" "$(dirname "${OUTPUT_HTML}")"

if [[ -n "${GAP_COUNT}" && -z "${GAP_DURATION_SEC}" ]]; then
  echo "GAP_COUNT and GAP_DURATION_SEC must be provided together." >&2
  exit 2
fi
if [[ -z "${GAP_COUNT}" && -n "${GAP_DURATION_SEC}" ]]; then
  echo "GAP_COUNT and GAP_DURATION_SEC must be provided together." >&2
  exit 2
fi

EXTRA_ARGS=()
if [[ -n "${GAP_COUNT}" && -n "${GAP_DURATION_SEC}" ]]; then
  EXTRA_ARGS+=(--gap-count "${GAP_COUNT}" --gap-duration-sec "${GAP_DURATION_SEC}")
fi

echo "Dataset: ${DATASET_DIR}"
echo "Output dir: ${OUTPUT_DIR}"
echo "Output HTML: ${OUTPUT_HTML}"
echo "Scenario mode: ${SCENARIO_MODE}"
echo "Custom outages: ${OUTAGES}"
if [[ -n "${GAP_COUNT}" && -n "${GAP_DURATION_SEC}" ]]; then
  echo "Gap group: ${GAP_COUNT} x ${GAP_DURATION_SEC}s"
else
  echo "Gap group: default 6 groups"
fi
echo "Duration sec: ${DURATION_SEC}"
echo "Cloud stride: ${CLOUD_STRIDE}"
echo "IMU gyro scale: ${IMU_GYRO_SCALE}"

python3 "${WORKSPACE_DIR}/scripts/offline_fuse_eval.py" \
  --dataset-dir "${DATASET_DIR}" \
  --output-dir "${OUTPUT_DIR}" \
  --output-html "${OUTPUT_HTML}" \
  --scenario-mode "${SCENARIO_MODE}" \
  --outages "${OUTAGES}" \
  --start-sec "${START_SEC}" \
  --duration-sec "${DURATION_SEC}" \
  --max-clouds "${MAX_CLOUDS}" \
  --cloud-stride "${CLOUD_STRIDE}" \
  --imu-gyro-scale "${IMU_GYRO_SCALE}" \
  "${EXTRA_ARGS[@]}" \
  "$@"

echo "Open result:"
echo "xdg-open ${OUTPUT_HTML}"
