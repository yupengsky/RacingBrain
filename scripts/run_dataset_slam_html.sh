#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

DATASET_DIR="${DATASET_DIR:-/media/yupeng/S11/rosbag2_2026_05_10-15_02_50}"
OUTPUT_HTML="${OUTPUT_HTML:-${WORKSPACE_DIR}/results/slam_mapping_2026_05_10-15_02_50.html}"
LOG_DIR="${LOG_DIR:-${WORKSPACE_DIR}/log/runtime/slam_html_$(date +%Y%m%d_%H%M%S)}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-231}"
BAG_RATE="${BAG_RATE:-0.5}"
STARTUP_WAIT="${STARTUP_WAIT:-8}"
RUN_DURATION_SEC="${RUN_DURATION_SEC:-90}"
FRAME_PERIOD_SEC="${FRAME_PERIOD_SEC:-0.5}"
MAX_LIDAR_POINTS="${MAX_LIDAR_POINTS:-900}"

if [[ ! -d "${DATASET_DIR}" ]]; then
  echo "Dataset directory not found: ${DATASET_DIR}" >&2
  exit 1
fi

if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  "${WORKSPACE_DIR}/scripts/build_ros_clean.sh" --base-paths \
    "${WORKSPACE_DIR}/LocalizationMapping/gnss/gnss_ins_msg" \
    "${WORKSPACE_DIR}/LocalizationMapping/slam/drd25_msgs" \
    "${WORKSPACE_DIR}/LocalizationMapping/slam/slam" \
    "${WORKSPACE_DIR}/LocalizationMapping/perception/src/cone_ws/src/cone_interfaces" \
    "${WORKSPACE_DIR}/LocalizationMapping/perception/src/cone_segmentation_test_3d/src/test_cone_segmentation" \
    "${WORKSPACE_DIR}/LocalizationMapping/perception/src/run_perception" \
    "${WORKSPACE_DIR}/LocalizationMapping/RacingBrain"
fi

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

mkdir -p "${LOG_DIR}" "$(dirname "${OUTPUT_HTML}")"
export ROS_DOMAIN_ID
export PYTHONUNBUFFERED=1

cleanup() {
  set +e
  for pid in ${BAG_PID:-} ${COLLECTOR_PID:-} ${STACK_PID:-}; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  sleep 2
  for pid in ${BAG_PID:-} ${COLLECTOR_PID:-} ${STACK_PID:-}; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

ros2 launch racingbrain localization_mapping.launch.py \
  "track:=acceleration" \
  "rviz:=false" \
  "lidar_backend:=cluster" \
  "fusion_mode:=lidar_only" \
  "mapping_gate:=false" \
  "enable_planning:=false" \
  "enable_health:=false" \
  "health_expected_perception:=false" \
  >"${LOG_DIR}/stack.log" 2>&1 &
STACK_PID=$!

sleep "${STARTUP_WAIT}"

if [[ "${RUN_DURATION_SEC}" == "full" ]]; then
  COLLECT_DURATION_SEC=86400
else
  COLLECT_DURATION_SEC="${RUN_DURATION_SEC}"
fi

python3 "${WORKSPACE_DIR}/scripts/collect_slam_html.py" \
  --output "${OUTPUT_HTML}" \
  --duration-sec "${COLLECT_DURATION_SEC}" \
  --frame-period-sec "${FRAME_PERIOD_SEC}" \
  --max-lidar-points "${MAX_LIDAR_POINTS}" \
  >"${LOG_DIR}/collector.log" 2>&1 &
COLLECTOR_PID=$!

sleep 1

set +e
if [[ "${RUN_DURATION_SEC}" == "full" ]]; then
  ros2 bag play "${DATASET_DIR}" \
      --rate "${BAG_RATE}" \
      --topics /lidar_points /gongji_gnss_ins_64 \
      >"${LOG_DIR}/bag.log" 2>&1
else
  timeout --signal=INT "${RUN_DURATION_SEC}" \
    ros2 bag play "${DATASET_DIR}" \
      --rate "${BAG_RATE}" \
      --topics /lidar_points /gongji_gnss_ins_64 \
      >"${LOG_DIR}/bag.log" 2>&1
fi
BAG_STATUS=$?
set -e
if [[ "${BAG_STATUS}" -ne 0 && "${BAG_STATUS}" -ne 124 && "${BAG_STATUS}" -ne 130 ]]; then
  echo "ros2 bag play exited with status ${BAG_STATUS}; preserving collected HTML." >&2
fi

if [[ -n "${COLLECTOR_PID:-}" ]] && kill -0 "${COLLECTOR_PID}" 2>/dev/null; then
  kill -INT "${COLLECTOR_PID}" 2>/dev/null || true
fi
wait "${COLLECTOR_PID}" || true
echo "${OUTPUT_HTML}"
if [[ "${BAG_STATUS}" -ne 0 && "${BAG_STATUS}" -ne 124 && "${BAG_STATUS}" -ne 130 ]]; then
  exit "${BAG_STATUS}"
fi
