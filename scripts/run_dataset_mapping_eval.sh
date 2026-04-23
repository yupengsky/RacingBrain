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
FAULT_PROFILE="${FAULT_PROFILE:-none}"
FAULT_START_SEC="${FAULT_START_SEC:-0.0}"
FAULT_DURATION_SEC="${FAULT_DURATION_SEC:--1.0}"
FAULT_CAMERA_MODE="${FAULT_CAMERA_MODE:-}"
FAULT_LIDAR_MODE="${FAULT_LIDAR_MODE:-}"
FAULT_GNSS_MODE="${FAULT_GNSS_MODE:-}"
FAULT_CAMERA_DROP_EVERY="${FAULT_CAMERA_DROP_EVERY:-}"
FAULT_LIDAR_DROP_EVERY="${FAULT_LIDAR_DROP_EVERY:-}"
FAULT_GNSS_DROP_EVERY="${FAULT_GNSS_DROP_EVERY:-}"
FAULT_CAMERA_BLUR_KERNEL="${FAULT_CAMERA_BLUR_KERNEL:-}"
FAULT_CAMERA_STAMP_OFFSET_SEC="${FAULT_CAMERA_STAMP_OFFSET_SEC:-}"
FAULT_LIDAR_STAMP_OFFSET_SEC="${FAULT_LIDAR_STAMP_OFFSET_SEC:-}"
FAULT_GNSS_STAMP_OFFSET_SEC="${FAULT_GNSS_STAMP_OFFSET_SEC:-}"
FAULT_CAMERA_PUBLISH_DELAY_SEC="${FAULT_CAMERA_PUBLISH_DELAY_SEC:-}"
FAULT_LIDAR_PUBLISH_DELAY_SEC="${FAULT_LIDAR_PUBLISH_DELAY_SEC:-}"
FAULT_GNSS_PUBLISH_DELAY_SEC="${FAULT_GNSS_PUBLISH_DELAY_SEC:-}"
FUSION_CALIBRATION_SOURCE="${FUSION_CALIBRATION_SOURCE:-${WORKSPACE_DIR}/LocalizationMapping/perception/src/fs_fusion_box/config/calibration.yaml}"
CALIB_BIAS_TX="${CALIB_BIAS_TX:-}"
CALIB_BIAS_TY="${CALIB_BIAS_TY:-}"
CALIB_BIAS_TZ="${CALIB_BIAS_TZ:-}"
CALIB_BIAS_ROLL_DEG="${CALIB_BIAS_ROLL_DEG:-}"
CALIB_BIAS_PITCH_DEG="${CALIB_BIAS_PITCH_DEG:-}"
CALIB_BIAS_YAW_DEG="${CALIB_BIAS_YAW_DEG:-}"
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
  for pid in ${BAG_PID:-} ${MONITOR_PID:-} ${INJECTOR_PID:-} ${STACK_PID:-}; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  sleep 2
  for pid in ${BAG_PID:-} ${MONITOR_PID:-} ${INJECTOR_PID:-} ${STACK_PID:-}; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

case "${FAULT_PROFILE}" in
  none)
    ;;
  camera_blank)
    FAULT_CAMERA_MODE="${FAULT_CAMERA_MODE:-blank}"
    ;;
  camera_blur)
    FAULT_CAMERA_MODE="${FAULT_CAMERA_MODE:-blur}"
    FAULT_CAMERA_BLUR_KERNEL="${FAULT_CAMERA_BLUR_KERNEL:-21}"
    ;;
  camera_dropout)
    FAULT_CAMERA_MODE="${FAULT_CAMERA_MODE:-drop}"
    FAULT_CAMERA_DROP_EVERY="${FAULT_CAMERA_DROP_EVERY:-2}"
    ;;
  lidar_stamp_skew)
    FAULT_LIDAR_STAMP_OFFSET_SEC="${FAULT_LIDAR_STAMP_OFFSET_SEC:-0.18}"
    ;;
  gnss_stamp_skew)
    FAULT_GNSS_STAMP_OFFSET_SEC="${FAULT_GNSS_STAMP_OFFSET_SEC:-0.20}"
    ;;
  fusion_calibration_bias)
    CALIB_BIAS_TX="${CALIB_BIAS_TX:-0.18}"
    CALIB_BIAS_TY="${CALIB_BIAS_TY:-0.05}"
    CALIB_BIAS_TZ="${CALIB_BIAS_TZ:-0.00}"
    CALIB_BIAS_ROLL_DEG="${CALIB_BIAS_ROLL_DEG:-0.0}"
    CALIB_BIAS_PITCH_DEG="${CALIB_BIAS_PITCH_DEG:-0.0}"
    CALIB_BIAS_YAW_DEG="${CALIB_BIAS_YAW_DEG:-6.0}"
    ;;
  *)
    echo "Unsupported FAULT_PROFILE: ${FAULT_PROFILE}" >&2
    exit 1
    ;;
esac

FAULT_CAMERA_MODE="${FAULT_CAMERA_MODE:-none}"
FAULT_LIDAR_MODE="${FAULT_LIDAR_MODE:-none}"
FAULT_GNSS_MODE="${FAULT_GNSS_MODE:-none}"
FAULT_CAMERA_DROP_EVERY="${FAULT_CAMERA_DROP_EVERY:-2}"
FAULT_LIDAR_DROP_EVERY="${FAULT_LIDAR_DROP_EVERY:-2}"
FAULT_GNSS_DROP_EVERY="${FAULT_GNSS_DROP_EVERY:-2}"
FAULT_CAMERA_BLUR_KERNEL="${FAULT_CAMERA_BLUR_KERNEL:-19}"
FAULT_CAMERA_STAMP_OFFSET_SEC="${FAULT_CAMERA_STAMP_OFFSET_SEC:-0.0}"
FAULT_LIDAR_STAMP_OFFSET_SEC="${FAULT_LIDAR_STAMP_OFFSET_SEC:-0.0}"
FAULT_GNSS_STAMP_OFFSET_SEC="${FAULT_GNSS_STAMP_OFFSET_SEC:-0.0}"
FAULT_CAMERA_PUBLISH_DELAY_SEC="${FAULT_CAMERA_PUBLISH_DELAY_SEC:-0.0}"
FAULT_LIDAR_PUBLISH_DELAY_SEC="${FAULT_LIDAR_PUBLISH_DELAY_SEC:-0.0}"
FAULT_GNSS_PUBLISH_DELAY_SEC="${FAULT_GNSS_PUBLISH_DELAY_SEC:-0.0}"

USE_FAULT_INJECTOR=false
if [[ "${FAULT_CAMERA_MODE}" != "none" || "${FAULT_LIDAR_MODE}" != "none" || "${FAULT_GNSS_MODE}" != "none" || \
      "${FAULT_CAMERA_STAMP_OFFSET_SEC}" != "0.0" || "${FAULT_LIDAR_STAMP_OFFSET_SEC}" != "0.0" || "${FAULT_GNSS_STAMP_OFFSET_SEC}" != "0.0" || \
      "${FAULT_CAMERA_PUBLISH_DELAY_SEC}" != "0.0" || "${FAULT_LIDAR_PUBLISH_DELAY_SEC}" != "0.0" || "${FAULT_GNSS_PUBLISH_DELAY_SEC}" != "0.0" ]]; then
  USE_FAULT_INJECTOR=true
fi

CAMERA_TOPIC="/camera1/image_raw"
LIDAR_TOPIC="/lidar_points"
GNSS_TOPIC="/gongji_gnss_ins_64"
if [[ "${USE_FAULT_INJECTOR}" == "true" ]]; then
  CAMERA_TOPIC="/fault_injected/camera1/image_raw"
  LIDAR_TOPIC="/fault_injected/lidar_points"
  GNSS_TOPIC="/fault_injected/gongji_gnss_ins_64"
fi

FUSION_CALIBRATION_FILE=""
if [[ "${FAULT_PROFILE}" == "fusion_calibration_bias" ]]; then
  FUSION_CALIBRATION_FILE="${LOG_DIR}/fault_calibration.yaml"
  python3 "${WORKSPACE_DIR}/scripts/eval/generate_fault_calibration.py" \
    --source "${FUSION_CALIBRATION_SOURCE}" \
    --output "${FUSION_CALIBRATION_FILE}" \
    --translation-x "${CALIB_BIAS_TX}" \
    --translation-y "${CALIB_BIAS_TY}" \
    --translation-z "${CALIB_BIAS_TZ}" \
    --roll-deg "${CALIB_BIAS_ROLL_DEG}" \
    --pitch-deg "${CALIB_BIAS_PITCH_DEG}" \
    --yaw-deg "${CALIB_BIAS_YAW_DEG}"
fi

export \
  FAULT_PROFILE \
  FAULT_START_SEC \
  FAULT_DURATION_SEC \
  USE_FAULT_INJECTOR \
  CAMERA_TOPIC \
  LIDAR_TOPIC \
  GNSS_TOPIC \
  FUSION_CALIBRATION_FILE \
  FAULT_CAMERA_MODE \
  FAULT_LIDAR_MODE \
  FAULT_GNSS_MODE \
  FAULT_CAMERA_DROP_EVERY \
  FAULT_LIDAR_DROP_EVERY \
  FAULT_GNSS_DROP_EVERY \
  FAULT_CAMERA_BLUR_KERNEL \
  FAULT_CAMERA_STAMP_OFFSET_SEC \
  FAULT_LIDAR_STAMP_OFFSET_SEC \
  FAULT_GNSS_STAMP_OFFSET_SEC \
  FAULT_CAMERA_PUBLISH_DELAY_SEC \
  FAULT_LIDAR_PUBLISH_DELAY_SEC \
  FAULT_GNSS_PUBLISH_DELAY_SEC \
  CALIB_BIAS_TX \
  CALIB_BIAS_TY \
  CALIB_BIAS_TZ \
  CALIB_BIAS_ROLL_DEG \
  CALIB_BIAS_PITCH_DEG \
  CALIB_BIAS_YAW_DEG

SCENARIO_FILE="${LOG_DIR}/scenario.json"
python3 - "${SCENARIO_FILE}" <<'PY'
import json
import os
import sys

scenario = {
    "profile": os.environ["FAULT_PROFILE"],
    "fault_start_sec": float(os.environ["FAULT_START_SEC"]),
    "fault_duration_sec": float(os.environ["FAULT_DURATION_SEC"]),
    "use_fault_injector": os.environ["USE_FAULT_INJECTOR"].lower() == "true",
    "camera_topic": os.environ["CAMERA_TOPIC"],
    "lidar_topic": os.environ["LIDAR_TOPIC"],
    "gnss_topic": os.environ["GNSS_TOPIC"],
    "fusion_calibration_file": os.environ.get("FUSION_CALIBRATION_FILE") or None,
    "camera_fault": {
        "mode": os.environ["FAULT_CAMERA_MODE"],
        "drop_every": int(os.environ["FAULT_CAMERA_DROP_EVERY"]),
        "blur_kernel": int(os.environ["FAULT_CAMERA_BLUR_KERNEL"]),
        "stamp_offset_sec": float(os.environ["FAULT_CAMERA_STAMP_OFFSET_SEC"]),
        "publish_delay_sec": float(os.environ["FAULT_CAMERA_PUBLISH_DELAY_SEC"]),
    },
    "lidar_fault": {
        "mode": os.environ["FAULT_LIDAR_MODE"],
        "drop_every": int(os.environ["FAULT_LIDAR_DROP_EVERY"]),
        "stamp_offset_sec": float(os.environ["FAULT_LIDAR_STAMP_OFFSET_SEC"]),
        "publish_delay_sec": float(os.environ["FAULT_LIDAR_PUBLISH_DELAY_SEC"]),
    },
    "gnss_fault": {
        "mode": os.environ["FAULT_GNSS_MODE"],
        "drop_every": int(os.environ["FAULT_GNSS_DROP_EVERY"]),
        "stamp_offset_sec": float(os.environ["FAULT_GNSS_STAMP_OFFSET_SEC"]),
        "publish_delay_sec": float(os.environ["FAULT_GNSS_PUBLISH_DELAY_SEC"]),
    },
    "calibration_bias": {
        "translation_x": float(os.environ.get("CALIB_BIAS_TX") or 0.0),
        "translation_y": float(os.environ.get("CALIB_BIAS_TY") or 0.0),
        "translation_z": float(os.environ.get("CALIB_BIAS_TZ") or 0.0),
        "roll_deg": float(os.environ.get("CALIB_BIAS_ROLL_DEG") or 0.0),
        "pitch_deg": float(os.environ.get("CALIB_BIAS_PITCH_DEG") or 0.0),
        "yaw_deg": float(os.environ.get("CALIB_BIAS_YAW_DEG") or 0.0),
    },
}
with open(sys.argv[1], "w", encoding="utf-8") as f:
    json.dump(scenario, f, indent=2, ensure_ascii=False)
PY

echo "Workspace: ${WORKSPACE_DIR}"
echo "Dataset: ${DATASET_DIR}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "Bag rate: ${BAG_RATE}"
echo "Track: ${TRACK}"
echo "LiDAR backend: ${LIDAR_BACKEND}"
echo "Fault profile: ${FAULT_PROFILE}"
echo "Fault injector: ${USE_FAULT_INJECTOR}"
echo "Eval timeout: ${EVAL_TIMEOUT}"
echo "Idle timeout: ${IDLE_TIMEOUT}"
echo "Duplicate threshold: ${DUPLICATE_THRESHOLD}"
echo "Logs: ${LOG_DIR}"

STACK_CMD=(
  ros2 launch racingbrain localization_mapping.launch.py
  "track:=${TRACK}"
  "rviz:=${RVIZ}"
  "lidar_backend:=${LIDAR_BACKEND}"
  "eval_debug:=true"
  "enable_planning:=false"
  "enable_health:=true"
  "camera_topic:=${CAMERA_TOPIC}"
  "lidar_topic:=${LIDAR_TOPIC}"
  "gnss_topic:=${GNSS_TOPIC}"
)
if [[ -n "${FUSION_CALIBRATION_FILE}" ]]; then
  STACK_CMD+=("fusion_calibration_file:=${FUSION_CALIBRATION_FILE}")
fi
"${STACK_CMD[@]}" >"${LOG_DIR}/stack.log" 2>&1 &
STACK_PID=$!

sleep "${STARTUP_WAIT}"

if [[ "${USE_FAULT_INJECTOR}" == "true" ]]; then
  python3 "${WORKSPACE_DIR}/scripts/eval/runtime_fault_injector.py" \
    --camera-mode "${FAULT_CAMERA_MODE}" \
    --lidar-mode "${FAULT_LIDAR_MODE}" \
    --gnss-mode "${FAULT_GNSS_MODE}" \
    --camera-drop-every "${FAULT_CAMERA_DROP_EVERY}" \
    --lidar-drop-every "${FAULT_LIDAR_DROP_EVERY}" \
    --gnss-drop-every "${FAULT_GNSS_DROP_EVERY}" \
    --camera-blur-kernel "${FAULT_CAMERA_BLUR_KERNEL}" \
    --camera-stamp-offset-sec "${FAULT_CAMERA_STAMP_OFFSET_SEC}" \
    --lidar-stamp-offset-sec "${FAULT_LIDAR_STAMP_OFFSET_SEC}" \
    --gnss-stamp-offset-sec "${FAULT_GNSS_STAMP_OFFSET_SEC}" \
    --camera-publish-delay-sec "${FAULT_CAMERA_PUBLISH_DELAY_SEC}" \
    --lidar-publish-delay-sec "${FAULT_LIDAR_PUBLISH_DELAY_SEC}" \
    --gnss-publish-delay-sec "${FAULT_GNSS_PUBLISH_DELAY_SEC}" \
    --fault-start-sec "${FAULT_START_SEC}" \
    --fault-duration-sec "${FAULT_DURATION_SEC}" \
    --log-path "${LOG_DIR}/fault_injector_stats.json" \
    >"${LOG_DIR}/fault_injector.log" 2>&1 &
  INJECTOR_PID=$!
  sleep 1
fi

python3 "${WORKSPACE_DIR}/scripts/eval/drd26_eval_monitor.py" \
  --log-dir "${LOG_DIR}" \
  --dataset "${DATASET_DIR}" \
  --scenario-file "${SCENARIO_FILE}" \
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
if [[ "${USE_FAULT_INJECTOR}" == "true" ]]; then
  echo
  echo "--- fault injector tail ---"
  tail -n 60 "${LOG_DIR}/fault_injector.log" || true
fi
echo
echo "--- bag tail ---"
tail -n 30 "${LOG_DIR}/bag.log" || true
echo
echo "Summary: ${LOG_DIR}/summary.json"
echo "Report: ${LOG_DIR}/report.md"

exit "${MONITOR_STATUS}"
