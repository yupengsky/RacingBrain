#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

VENV_DIR="${VENV_DIR:-${WORKSPACE_DIR}/.venv_ros_ml}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-226}"
BAG_RATE="${BAG_RATE:-0.25}"
STARTUP_WAIT="${STARTUP_WAIT:-12}"
MONITOR_TIMEOUT="${MONITOR_TIMEOUT:-150}"
TRACK="${TRACK:-acceleration}"
RVIZ="${RVIZ:-false}"
KEEP_RUNNING="${KEEP_RUNNING:-false}"
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

LOG_DIR="${LOG_DIR:-${WORKSPACE_DIR}/log/runtime/dataset_mapping_chain_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${LOG_DIR}"

cleanup() {
  set +e
  for pid in ${BAG_PID:-} ${STACK_PID:-}; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  sleep 2
  for pid in ${BAG_PID:-} ${STACK_PID:-}; do
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
echo "LiDAR backend: ${LIDAR_BACKEND}"
echo "Keep running after success: ${KEEP_RUNNING}"
echo "Logs: ${LOG_DIR}"

ros2 launch racingbrain localization_mapping.launch.py \
  "track:=${TRACK}" \
  "rviz:=${RVIZ}" \
  "lidar_backend:=${LIDAR_BACKEND}" \
  "enable_planning:=false" \
  "enable_health:=true" \
  >"${LOG_DIR}/stack.log" 2>&1 &
STACK_PID=$!

sleep "${STARTUP_WAIT}"

ros2 bag play "${DATASET_DIR}" \
  --rate "${BAG_RATE}" \
  --topics /camera1/image_raw /lidar_points /gongji_gnss_ins_64 \
  >"${LOG_DIR}/bag.log" 2>&1 &
BAG_PID=$!

set +e
python3 - "${LOG_DIR}" "${MONITOR_TIMEOUT}" <<'PY'
import json
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from gnss_ins_msg.msg import Gnssins64
from cone_interfaces.msg import ConeArray
from test_cone_segmentation.msg import ThreeDConeArray
from drd25_msgs.msg import Map
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


class ChainMonitor(Node):
    def __init__(self):
        super().__init__("racingbrain_mapping_chain_monitor")
        self.start = time.time()
        self.counts = {
            "/camera1/image_raw": 0,
            "/lidar_points": 0,
            "/gongji_gnss_ins_64": 0,
            "/yolo/cones": 0,
            "/cone_detection_custom": 0,
            "/perception/fusion/map": 0,
            "/global_map": 0,
            "/racingbrain/health/system": 0,
            "/racingbrain/perception/failure_state": 0,
        }
        self.first_times = {}
        self.max_yolo_cones = 0
        self.max_lidar_cones = 0
        self.max_fused_cones = 0
        self.max_global_markers = 0
        self.nonempty_global_messages = 0
        self.last_fused_stamp = None
        self.last_global_stamp = None
        self.last_health_status = None
        self.last_health_alert_count = 0
        self.last_health_alerts = []
        self.last_active_lidar_backend = None
        self.last_learning_failed = None
        self.last_failure_reasons = []

        self.create_subscription(Image, "/camera1/image_raw", self.cb("/camera1/image_raw"), 10)
        self.create_subscription(PointCloud2, "/lidar_points", self.cb("/lidar_points"), 10)
        self.create_subscription(Gnssins64, "/gongji_gnss_ins_64", self.cb("/gongji_gnss_ins_64"), 10)
        self.create_subscription(ConeArray, "/yolo/cones", self.cb_yolo, 10)
        self.create_subscription(ThreeDConeArray, "/cone_detection_custom", self.cb_lidar_cones, 10)
        self.create_subscription(Map, "/perception/fusion/map", self.cb_fusion, 10)
        self.create_subscription(MarkerArray, "/global_map", self.cb_global, 10)
        self.create_subscription(String, "/racingbrain/health/system", self.cb_health, 10)
        self.create_subscription(String, "/racingbrain/perception/failure_state", self.cb_failure_state, 10)

    def mark(self, topic):
        self.counts[topic] += 1
        self.first_times.setdefault(topic, round(time.time() - self.start, 3))

    def cb(self, topic):
        def wrapped(_msg):
            self.mark(topic)
        return wrapped

    def cb_yolo(self, msg):
        self.mark("/yolo/cones")
        self.max_yolo_cones = max(self.max_yolo_cones, len(msg.cones))

    def cb_lidar_cones(self, msg):
        self.mark("/cone_detection_custom")
        self.max_lidar_cones = max(self.max_lidar_cones, len(msg.cones))

    def cb_fusion(self, msg):
        self.mark("/perception/fusion/map")
        self.max_fused_cones = max(self.max_fused_cones, len(msg.track))
        self.last_fused_stamp = {
            "sec": msg.header.stamp.sec,
            "nanosec": msg.header.stamp.nanosec,
        }

    def cb_global(self, msg):
        self.mark("/global_map")
        self.max_global_markers = max(self.max_global_markers, len(msg.markers))
        if len(msg.markers) > 1:
            self.nonempty_global_messages += 1
            self.last_global_stamp = {
                "sec": msg.markers[0].header.stamp.sec,
                "nanosec": msg.markers[0].header.stamp.nanosec,
            }

    def cb_health(self, msg):
        self.mark("/racingbrain/health/system")
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.last_health_status = payload.get("overall_status")
        alerts = payload.get("alerts") or []
        if isinstance(alerts, list):
            self.last_health_alert_count = len(alerts)
            self.last_health_alerts = alerts
        else:
            self.last_health_alert_count = 0
            self.last_health_alerts = []

    def cb_failure_state(self, msg):
        self.mark("/racingbrain/perception/failure_state")
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.last_active_lidar_backend = payload.get("active_lidar_backend")
        self.last_learning_failed = payload.get("learning_failed")
        reasons = payload.get("failure_reasons") or []
        self.last_failure_reasons = reasons if isinstance(reasons, list) else []


log_dir = Path(sys.argv[1])
timeout_sec = float(sys.argv[2])

rclpy.init()
node = ChainMonitor()
deadline = time.time() + timeout_sec
success_since = None

while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    fused_ok = node.counts["/perception/fusion/map"] > 0 and node.max_fused_cones > 0
    global_ok = node.nonempty_global_messages > 0
    health_ok = node.counts["/racingbrain/health/system"] > 0
    if fused_ok and global_ok and health_ok:
        if success_since is None:
            success_since = time.time()
        if time.time() - success_since >= 8.0:
            break
    else:
        success_since = None

summary = {
    "success": node.counts["/perception/fusion/map"] > 0
    and node.max_fused_cones > 0
    and node.nonempty_global_messages > 0
    and node.counts["/racingbrain/health/system"] > 0,
    "elapsed_sec": round(time.time() - node.start, 3),
    "counts": node.counts,
    "first_times_sec": node.first_times,
    "max_yolo_cones": node.max_yolo_cones,
    "max_lidar_cones": node.max_lidar_cones,
    "max_fused_cones": node.max_fused_cones,
    "max_global_markers": node.max_global_markers,
    "nonempty_global_messages": node.nonempty_global_messages,
    "last_fused_stamp": node.last_fused_stamp,
    "last_global_stamp": node.last_global_stamp,
    "last_health_status": node.last_health_status,
    "last_health_alert_count": node.last_health_alert_count,
    "last_health_alerts": node.last_health_alerts,
    "last_active_lidar_backend": node.last_active_lidar_backend,
    "last_learning_failed": node.last_learning_failed,
    "last_failure_reasons": node.last_failure_reasons,
}

(log_dir / "summary.json").write_text(
    json.dumps(summary, indent=2, ensure_ascii=False),
    encoding="utf-8",
)
print(json.dumps(summary, indent=2, ensure_ascii=False))

node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if summary["success"] else 2)
PY
MONITOR_STATUS=$?
set -e

echo
echo "--- stack tail ---"
tail -n 80 "${LOG_DIR}/stack.log" || true
echo
echo "--- bag tail ---"
tail -n 40 "${LOG_DIR}/bag.log" || true
echo
echo "Summary: ${LOG_DIR}/summary.json"

if [[ "${MONITOR_STATUS}" -eq 0 && "${KEEP_RUNNING}" == "true" ]]; then
  echo
  echo "Chain is verified and still running. Press Ctrl-C to stop."
  wait "${BAG_PID}"
fi

exit "${MONITOR_STATUS}"
