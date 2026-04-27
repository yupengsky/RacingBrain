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
GATE_VARIANTS="${GATE_VARIANTS:-true}"

if [[ -n "${SCENARIOS:-}" ]]; then
  # shellcheck disable=SC2206
  SCENARIO_LIST=(${SCENARIOS})
else
  SCENARIO_LIST=(none camera_blank camera_blur lidar_stamp_skew gnss_stamp_skew fusion_calibration_bias)
fi

# shellcheck disable=SC2206
GATE_VARIANT_LIST=(${GATE_VARIANTS})

BENCHMARK_DIR="${BENCHMARK_DIR:-${WORKSPACE_DIR}/log/benchmark/dataset_fault_benchmark_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${BENCHMARK_DIR}"

echo "Workspace: ${WORKSPACE_DIR}"
echo "Benchmark dir: ${BENCHMARK_DIR}"
echo "Scenarios: ${SCENARIO_LIST[*]}"
echo "LiDAR backend: ${LIDAR_BACKEND}"
echo "Mapping gate variants: ${GATE_VARIANT_LIST[*]}"

overall_status=0

for scenario in "${SCENARIO_LIST[@]}"; do
  for mapping_gate in "${GATE_VARIANT_LIST[@]}"; do
    if [[ "${#GATE_VARIANT_LIST[@]}" -eq 1 ]]; then
      scenario_dir="${BENCHMARK_DIR}/${scenario}"
    else
      scenario_dir="${BENCHMARK_DIR}/${scenario}/mapping_gate_${mapping_gate}"
    fi
    mkdir -p "${scenario_dir}"
    echo
    echo "=== Running scenario: ${scenario} | mapping_gate=${mapping_gate} ==="
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
    MAPPING_GATE="${mapping_gate}" \
    FAULT_PROFILE="${scenario}" \
    "${WORKSPACE_DIR}/scripts/run_dataset_mapping_eval.sh" \
      >"${scenario_dir}/runner.log" 2>&1
    status=$?
    set -e
    echo "Scenario ${scenario} mapping_gate=${mapping_gate} exit code: ${status}"
    if [[ "${status}" -ne 0 ]]; then
      overall_status=1
    fi
  done
done

python3 - "${BENCHMARK_DIR}" "${#GATE_VARIANT_LIST[@]}" "${GATE_VARIANT_LIST[@]}" -- "${SCENARIO_LIST[@]}" <<'PY'
import csv
import json
import sys
from pathlib import Path

benchmark_dir = Path(sys.argv[1])
gate_count = int(sys.argv[2])
gate_variants = sys.argv[3 : 3 + gate_count]
separator_index = 3 + gate_count
if sys.argv[separator_index] != "--":
    raise SystemExit("internal argument error: missing -- separator")
scenarios = sys.argv[separator_index + 1 :]
rows = []

def summary_path_for(scenario, mapping_gate):
    if len(gate_variants) == 1:
        return benchmark_dir / scenario / "summary.json"
    return benchmark_dir / scenario / f"mapping_gate_{mapping_gate}" / "summary.json"

for scenario in scenarios:
  for mapping_gate in gate_variants:
    summary_path = summary_path_for(scenario, mapping_gate)
    row = {
        "scenario": scenario,
        "mapping_gate": mapping_gate,
        "summary_path": str(summary_path),
        "success": False,
        "last_health_status": None,
        "active_lidar_backend": None,
        "learning_failed": None,
        "final_stable_cones": None,
        "final_duplicate_pairs": None,
        "created_cones_total": None,
        "candidate_residue_total": None,
        "candidate_residue_per_frame": None,
        "stable_creation_ratio": None,
        "removal_churn_ratio": None,
        "unknown_observation_ratio": None,
        "final_duplicate_density": None,
        "risk_gate_rejected_new_cones": None,
        "risk_gate_downweighted_observations": None,
        "map_stability_score": None,
        "fused_unknown_ratio_mean": None,
        "fusion_consistency_score_mean": None,
        "fusion_calibration_drift_score_mean": None,
        "fusion_projection_error_px_mean": None,
        "fusion_stamp_delta_ms_mean": None,
        "lidar_total_ms_mean": None,
        "fusion_total_ms_mean": None,
        "mapping_sync_callback_ms_mean": None,
    }
    if summary_path.exists():
        data = json.loads(summary_path.read_text(encoding="utf-8"))
        processing = data.get("processing_time_ms", {})
        fusion_consistency = data.get("fusion_consistency", {})
        map_metrics = data.get("map") or {}
        map_pollution = data.get("map_pollution") or {}
        system_health = data.get("system_health") or {}
        last_health = system_health.get("last_non_stale") or system_health.get("last") or {}
        perception_failure = data.get("perception_failure") or {}
        last_failure = perception_failure.get("last_live") or perception_failure.get("last") or {}
        lidar_metrics = processing.get("lidar_cluster") or processing.get("pointpillars") or {}
        mapping_metrics = processing.get("mapping") or {}
        row.update(
            {
                "success": bool(data.get("success")),
                "last_health_status": last_health.get("overall_status"),
                "active_lidar_backend": last_failure.get("active_lidar_backend"),
                "learning_failed": last_failure.get("learning_failed"),
                "final_stable_cones": map_metrics.get("final_stable_cones"),
                "final_duplicate_pairs": map_metrics.get("final_duplicate_pairs"),
                "created_cones_total": map_pollution.get("created_cones_total"),
                "candidate_residue_total": map_pollution.get("candidate_residue_total"),
                "candidate_residue_per_frame": map_pollution.get("candidate_residue_per_frame"),
                "stable_creation_ratio": map_pollution.get("stable_creation_ratio"),
                "removal_churn_ratio": map_pollution.get("removal_churn_ratio"),
                "unknown_observation_ratio": map_pollution.get("unknown_observation_ratio"),
                "final_duplicate_density": map_pollution.get("final_duplicate_density"),
                "risk_gate_rejected_new_cones": map_pollution.get("risk_gate_rejected_new_cones"),
                "risk_gate_downweighted_observations": map_pollution.get("risk_gate_downweighted_observations"),
                "map_stability_score": map_pollution.get("map_stability_score"),
                "fused_unknown_ratio_mean": ((data.get("perception") or {}).get("fused_unknown_ratio") or {}).get("mean"),
                "fusion_consistency_score_mean": (fusion_consistency.get("consistency_score") or {}).get("mean"),
                "fusion_calibration_drift_score_mean": (fusion_consistency.get("calibration_drift_score") or {}).get("mean"),
                "fusion_projection_error_px_mean": (fusion_consistency.get("mean_nearest_camera_error_px") or {}).get("mean"),
                "fusion_stamp_delta_ms_mean": (fusion_consistency.get("abs_camera_lidar_stamp_delta_ms") or {}).get("mean"),
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

comparison_rows = []
for scenario in scenarios:
    by_gate = {row["mapping_gate"]: row for row in rows if row["scenario"] == scenario}
    on = by_gate.get("true")
    off = by_gate.get("false")
    if not on or not off:
        continue

    def delta(key):
        lhs = on.get(key)
        rhs = off.get(key)
        if lhs is None or rhs is None:
            return None
        try:
            return float(lhs) - float(rhs)
        except (TypeError, ValueError):
            return None

    comparison_rows.append(
        {
            "scenario": scenario,
            "success_gate_on": on.get("success"),
            "success_gate_off": off.get("success"),
            "stable_cones_delta_on_minus_off": delta("final_stable_cones"),
            "duplicate_pairs_delta_on_minus_off": delta("final_duplicate_pairs"),
            "candidate_residue_delta_on_minus_off": delta("candidate_residue_total"),
            "created_cones_delta_on_minus_off": delta("created_cones_total"),
            "stability_score_delta_on_minus_off": delta("map_stability_score"),
            "downweighted_observations_gate_on": on.get("risk_gate_downweighted_observations"),
            "rejected_new_cones_gate_on": on.get("risk_gate_rejected_new_cones"),
        }
    )

comparison_path = benchmark_dir / "mapping_gate_comparison.csv"
if comparison_rows:
    with comparison_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(comparison_rows[0].keys()))
        writer.writeheader()
        writer.writerows(comparison_rows)

lines = [
    "# RacingBrain Fault Benchmark",
    "",
    f"- Scenarios: `{', '.join(scenarios)}`",
    f"- Mapping gate variants: `{', '.join(gate_variants)}`",
    f"- Summary CSV: `{csv_path}`",
    f"- Gate comparison CSV: `{comparison_path if comparison_rows else 'n/a'}`",
    "",
    "| Scenario | Gate | Success | Health | Backend | Learning Failed | Stable Cones | Duplicates | Candidate Residue | Stability Score | UNKNOWN Mean | Consistency | Drift |",
    "|---|---|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|",
]

def fmt(value):
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)

for row in rows:
    lines.append(
        "| {scenario} | {mapping_gate} | {success} | {last_health_status} | {active_lidar_backend} | {learning_failed} | {final_stable_cones} | {final_duplicate_pairs} | {candidate_residue_total} | {map_stability_score} | {fused_unknown_ratio_mean} | {fusion_consistency_score_mean} | {fusion_calibration_drift_score_mean} |".format(
            **{k: fmt(v) for k, v in row.items()}
        )
    )

if comparison_rows:
    lines.extend(
        [
            "",
            "## Mapping Gate Delta",
            "",
            "| Scenario | Stable Delta | Duplicate Delta | Candidate Residue Delta | Stability Score Delta | Downweighted On | Rejected On |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in comparison_rows:
        lines.append(
            "| {scenario} | {stable_cones_delta_on_minus_off} | {duplicate_pairs_delta_on_minus_off} | {candidate_residue_delta_on_minus_off} | {stability_score_delta_on_minus_off} | {downweighted_observations_gate_on} | {rejected_new_cones_gate_on} |".format(
                **{k: fmt(v) for k, v in row.items()}
            )
        )

(benchmark_dir / "benchmark_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
PY

echo
echo "Benchmark summary: ${BENCHMARK_DIR}/benchmark_summary.csv"
echo "Benchmark report: ${BENCHMARK_DIR}/benchmark_report.md"

exit "${overall_status}"
