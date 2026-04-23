#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-226}"
BAG_RATE="${BAG_RATE:-0.5}"
STARTUP_WAIT="${STARTUP_WAIT:-10}"
EVAL_TIMEOUT="${EVAL_TIMEOUT:-90}"
IDLE_TIMEOUT="${IDLE_TIMEOUT:-8}"
TRACK="${TRACK:-acceleration}"
RVIZ="${RVIZ:-false}"
DUPLICATE_THRESHOLD="${DUPLICATE_THRESHOLD:-0.75}"
LIDAR_BACKEND="${LIDAR_BACKEND:-cluster}"

if [[ -n "${SCENARIOS:-}" ]]; then
  # shellcheck disable=SC2206
  SCENARIO_LIST=(${SCENARIOS})
else
  SCENARIO_LIST=(none camera_blank camera_blur lidar_stamp_skew gnss_stamp_skew fusion_calibration_bias)
fi

BENCHMARK_DIR="${BENCHMARK_DIR:-${WORKSPACE_DIR}/log/benchmark/dataset_fault_benchmark_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${BENCHMARK_DIR}"

echo "Workspace: ${WORKSPACE_DIR}"
echo "Benchmark dir: ${BENCHMARK_DIR}"
echo "Scenarios: ${SCENARIO_LIST[*]}"
echo "LiDAR backend: ${LIDAR_BACKEND}"

overall_status=0

for scenario in "${SCENARIO_LIST[@]}"; do
  scenario_dir="${BENCHMARK_DIR}/${scenario}"
  mkdir -p "${scenario_dir}"
  echo
  echo "=== Running scenario: ${scenario} ==="
  set +e
  LOG_DIR="${scenario_dir}" \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  BAG_RATE="${BAG_RATE}" \
  STARTUP_WAIT="${STARTUP_WAIT}" \
  EVAL_TIMEOUT="${EVAL_TIMEOUT}" \
  IDLE_TIMEOUT="${IDLE_TIMEOUT}" \
  TRACK="${TRACK}" \
  RVIZ="${RVIZ}" \
  DUPLICATE_THRESHOLD="${DUPLICATE_THRESHOLD}" \
  LIDAR_BACKEND="${LIDAR_BACKEND}" \
  FAULT_PROFILE="${scenario}" \
  "${WORKSPACE_DIR}/scripts/run_dataset_mapping_eval.sh" \
    >"${scenario_dir}/runner.log" 2>&1
  status=$?
  set -e
  echo "Scenario ${scenario} exit code: ${status}"
  if [[ "${status}" -ne 0 ]]; then
    overall_status=1
  fi
done

python3 - "${BENCHMARK_DIR}" "${SCENARIO_LIST[@]}" <<'PY'
import csv
import json
import sys
from pathlib import Path

benchmark_dir = Path(sys.argv[1])
scenarios = sys.argv[2:]
rows = []

for scenario in scenarios:
    summary_path = benchmark_dir / scenario / "summary.json"
    row = {
        "scenario": scenario,
        "summary_path": str(summary_path),
        "success": False,
        "last_health_status": None,
        "final_stable_cones": None,
        "fused_unknown_ratio_mean": None,
        "lidar_total_ms_mean": None,
        "fusion_total_ms_mean": None,
        "mapping_sync_callback_ms_mean": None,
    }
    if summary_path.exists():
        data = json.loads(summary_path.read_text(encoding="utf-8"))
        processing = data.get("processing_time_ms", {})
        lidar_metrics = processing.get("lidar_cluster") or processing.get("pointpillars") or {}
        mapping_metrics = processing.get("mapping") or {}
        row.update(
            {
                "success": bool(data.get("success")),
                "last_health_status": ((data.get("system_health") or {}).get("last") or {}).get("overall_status"),
                "final_stable_cones": ((data.get("map") or {}).get("final_stable_cones")),
                "fused_unknown_ratio_mean": ((data.get("perception") or {}).get("fused_unknown_ratio") or {}).get("mean"),
                "lidar_total_ms_mean": (lidar_metrics.get("total_ms") or {}).get("mean"),
                "fusion_total_ms_mean": ((processing.get("fusion") or {}).get("total_ms") or {}).get("mean"),
                "mapping_sync_callback_ms_mean": (mapping_metrics.get("sync_callback_ms") or {}).get("mean"),
            }
        )
    rows.append(row)

csv_path = benchmark_dir / "benchmark_summary.csv"
with csv_path.open("w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    writer.writeheader()
    writer.writerows(rows)

lines = [
    "# RacingBrain Fault Benchmark",
    "",
    f"- Scenarios: `{', '.join(scenarios)}`",
    f"- Summary CSV: `{csv_path}`",
    "",
    "| Scenario | Success | Health | Stable Cones | Fused UNKNOWN Mean | LiDAR Mean ms | Fusion Mean ms | Mapping Sync Mean ms |",
    "|---|---|---|---:|---:|---:|---:|---:|",
]

def fmt(value):
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)

for row in rows:
    lines.append(
        "| {scenario} | {success} | {last_health_status} | {final_stable_cones} | {fused_unknown_ratio_mean} | {lidar_total_ms_mean} | {fusion_total_ms_mean} | {mapping_sync_callback_ms_mean} |".format(
            **{k: fmt(v) for k, v in row.items()}
        )
    )

(benchmark_dir / "benchmark_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
PY

echo
echo "Benchmark summary: ${BENCHMARK_DIR}/benchmark_summary.csv"
echo "Benchmark report: ${BENCHMARK_DIR}/benchmark_report.md"

exit "${overall_status}"
