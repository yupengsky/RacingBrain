#!/usr/bin/env python3
"""Summarize RacingBrain replay, fault, and camera benchmark artifacts.

The evaluator already writes rich per-run JSON/CSV files. This script turns
those scattered run folders into paper-facing tables without adding runtime
dependencies or changing the online ROS stack.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from statistics import fmean
from typing import Any, Dict, Iterable, List, Optional, Sequence


DEFAULT_OUTPUT_DIR = Path("log/benchmark/benchmark_summary_latest")

TOPICS = {
    "camera_hz": "/camera1/image_raw",
    "lidar_points_hz": "/lidar_points",
    "yolo_hz": "/yolo/cones",
    "lidar_cones_hz": "/cone_detection_custom",
    "fusion_map_hz": "/perception/fusion/map",
    "global_map_hz": "/global_map",
    "health_hz": "/racingbrain/health/system",
    "planning_graph_hz": "/planning/track_graph",
}

SUMMARY_FIELDS = [
    "run_name",
    "run_type",
    "run_dir",
    "success",
    "profile",
    "backend_requested",
    "backend_effective",
    "mapping_gate",
    "lidar_verifier",
    "duration_sec",
    "elapsed_wall_sec",
    "camera_hz",
    "lidar_points_hz",
    "yolo_hz",
    "lidar_cones_hz",
    "fusion_map_hz",
    "global_map_hz",
    "health_hz",
    "planning_graph_hz",
    "pointpillars_total_mean_ms",
    "pointpillars_total_p95_ms",
    "cluster_total_mean_ms",
    "cluster_total_p95_ms",
    "yolo_inference_mean_ms",
    "yolo_inference_p95_ms",
    "fusion_total_mean_ms",
    "fusion_total_p95_ms",
    "mapping_sync_mean_ms",
    "mapping_sync_p95_ms",
    "camera_yolo_gpu_mean_ms",
    "camera_yolo_gpu_p95_ms",
    "camera_classical_cone_mean_ms",
    "camera_classical_cone_p95_ms",
    "camera_yolo_boxes_mean",
    "camera_classical_candidates_mean",
    "final_stable_cones",
    "final_duplicate_pairs",
    "final_candidate_cones",
    "created_cones_total",
    "candidate_residue_total",
    "stable_creation_ratio",
    "removal_churn_ratio",
    "unknown_observation_ratio",
    "final_duplicate_density",
    "risk_gate_rejected_new_cones",
    "risk_gate_downweighted_observations",
    "map_stability_score",
    "fused_unknown_ratio_mean",
    "fusion_consistency_mean",
    "fusion_calibration_drift_mean",
    "fusion_projection_error_px_mean",
    "fusion_stamp_delta_ms_mean",
    "runtime_budget_states",
    "runtime_budget_total_p95_mean_ms",
    "task_risk_states",
    "task_risk_score_mean",
    "last_write_policy",
    "planning_frames",
    "planning_ready_frames",
    "planning_ready_ratio",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-root",
        action="append",
        type=Path,
        default=None,
        help="Root directory to scan for summary.json files. Can be repeated. Defaults to log/eval.",
    )
    parser.add_argument(
        "--benchmark-root",
        action="append",
        type=Path,
        default=None,
        help="Root directory to scan for mapping_gate_comparison.csv files. Defaults to log/benchmark.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"Directory for generated tables. Defaults to {DEFAULT_OUTPUT_DIR}.",
    )
    parser.add_argument(
        "--max-markdown-runs",
        type=int,
        default=80,
        help="Maximum detailed runs to include in the Markdown table.",
    )
    return parser.parse_args()


def get_path(data: Any, path: Sequence[str], default: Any = None) -> Any:
    current = data
    for part in path:
        if not isinstance(current, dict) or part not in current:
            return default
        current = current[part]
    return current


def stat(data: Any, path: Sequence[str], key: str = "mean") -> Any:
    value = get_path(data, path)
    if isinstance(value, dict):
        return value.get(key)
    return None


def number(value: Any) -> Optional[float]:
    if value is None or value == "":
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(out):
        return None
    return out


def fmt(value: Any, digits: int = 3) -> str:
    if value is None or value == "":
        return "n/a"
    if isinstance(value, bool):
        return "true" if value else "false"
    numeric = number(value)
    if numeric is not None:
        if abs(numeric - round(numeric)) < 1e-9 and abs(numeric) < 100000:
            return str(int(round(numeric)))
        return f"{numeric:.{digits}f}"
    return str(value)


def compact_counts(counts: Any) -> str:
    if not isinstance(counts, dict) or not counts:
        return ""
    return ";".join(f"{key}:{counts[key]}" for key in sorted(counts))


def discover_summaries(roots: Iterable[Path]) -> List[Path]:
    summaries: List[Path] = []
    for root in roots:
        if root.is_file() and root.name == "summary.json":
            summaries.append(root)
        elif root.exists():
            summaries.extend(root.rglob("summary.json"))
    return sorted(set(summaries), key=lambda p: (p.stat().st_mtime, str(p)), reverse=True)


def discover_gate_comparisons(roots: Iterable[Path]) -> List[Path]:
    paths: List[Path] = []
    for root in roots:
        if root.is_file() and root.name == "mapping_gate_comparison.csv":
            paths.append(root)
        elif root.exists():
            paths.extend(root.rglob("mapping_gate_comparison.csv"))
    return sorted(set(paths), key=lambda p: (p.stat().st_mtime, str(p)), reverse=True)


def infer_run_type(data: Dict[str, Any]) -> str:
    if "yolo_gpu_ms" in data or "classical_cone_cpu_ms" in data:
        return "camera_detector_benchmark"
    if data.get("scenario") or data.get("map_pollution") or data.get("runtime_budget"):
        return "mapping_replay"
    return "generic_eval"


def topic_rate(data: Dict[str, Any], topic_name: str) -> Any:
    topic = get_path(data, ["topics", topic_name])
    if not isinstance(topic, dict):
        return None
    return topic.get("wall_rate_hz") or topic.get("stamp_rate_hz")


def ready_count(data: Dict[str, Any]) -> Optional[int]:
    counts = get_path(data, ["planning", "ready_counts"], {})
    if not isinstance(counts, dict):
        return None
    total = 0
    for key, value in counts.items():
        if str(key).lower() == "true":
            try:
                total += int(value)
            except (TypeError, ValueError):
                pass
    return total


def collect_row(path: Path) -> Dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    scenario = data.get("scenario") if isinstance(data.get("scenario"), dict) else {}
    lidar_backend = data.get("lidar_backend") if isinstance(data.get("lidar_backend"), dict) else {}
    processing = data.get("processing_time_ms") if isinstance(data.get("processing_time_ms"), dict) else {}
    run_type = infer_run_type(data)

    backend_requested = scenario.get("lidar_backend") or lidar_backend.get("requested")
    backend_effective = lidar_backend.get("effective") or backend_requested
    if not backend_effective:
        if "pointpillars" in processing:
            backend_effective = "pointpillars"
        elif "lidar_cluster" in processing:
            backend_effective = "cluster"

    row: Dict[str, Any] = {
        "run_name": path.parent.name,
        "run_type": run_type,
        "run_dir": str(path.parent),
        "success": data.get("success"),
        "profile": scenario.get("profile") or ("camera" if run_type == "camera_detector_benchmark" else ""),
        "backend_requested": backend_requested,
        "backend_effective": backend_effective,
        "mapping_gate": scenario.get("mapping_gate"),
        "lidar_verifier": scenario.get("lidar_verifier"),
        "duration_sec": get_path(data, ["bag_metadata", "duration_sec"]),
        "elapsed_wall_sec": data.get("elapsed_wall_sec"),
        "pointpillars_total_mean_ms": stat(processing, ["pointpillars", "total_ms"]),
        "pointpillars_total_p95_ms": stat(processing, ["pointpillars", "total_ms"], "p95"),
        "cluster_total_mean_ms": stat(processing, ["lidar_cluster", "total_ms"]),
        "cluster_total_p95_ms": stat(processing, ["lidar_cluster", "total_ms"], "p95"),
        "yolo_inference_mean_ms": stat(processing, ["yolo", "inference_ms"]),
        "yolo_inference_p95_ms": stat(processing, ["yolo", "inference_ms"], "p95"),
        "fusion_total_mean_ms": stat(processing, ["fusion", "total_ms"]),
        "fusion_total_p95_ms": stat(processing, ["fusion", "total_ms"], "p95"),
        "mapping_sync_mean_ms": stat(processing, ["mapping", "sync_callback_ms"]),
        "mapping_sync_p95_ms": stat(processing, ["mapping", "sync_callback_ms"], "p95"),
        "camera_yolo_gpu_mean_ms": stat(data, ["yolo_gpu_ms"]),
        "camera_yolo_gpu_p95_ms": stat(data, ["yolo_gpu_ms"], "p95"),
        "camera_classical_cone_mean_ms": stat(data, ["classical_cone_cpu_ms"]),
        "camera_classical_cone_p95_ms": stat(data, ["classical_cone_cpu_ms"], "p95"),
        "camera_yolo_boxes_mean": stat(data, ["yolo_boxes_per_frame"]),
        "camera_classical_candidates_mean": stat(data, ["classical_cones_per_frame"]),
        "final_stable_cones": get_path(data, ["map", "final_stable_cones"]),
        "final_duplicate_pairs": get_path(data, ["map", "final_duplicate_pairs"]),
        "final_candidate_cones": get_path(data, ["map_layers", "final_candidate_cones"]),
        "created_cones_total": get_path(data, ["map_pollution", "created_cones_total"]),
        "candidate_residue_total": get_path(data, ["map_pollution", "candidate_residue_total"]),
        "stable_creation_ratio": get_path(data, ["map_pollution", "stable_creation_ratio"]),
        "removal_churn_ratio": get_path(data, ["map_pollution", "removal_churn_ratio"]),
        "unknown_observation_ratio": get_path(data, ["map_pollution", "unknown_observation_ratio"]),
        "final_duplicate_density": get_path(data, ["map_pollution", "final_duplicate_density"]),
        "risk_gate_rejected_new_cones": get_path(data, ["map_pollution", "risk_gate_rejected_new_cones"]),
        "risk_gate_downweighted_observations": get_path(data, ["map_pollution", "risk_gate_downweighted_observations"]),
        "map_stability_score": get_path(data, ["map_pollution", "map_stability_score"]),
        "fused_unknown_ratio_mean": stat(data, ["perception", "fused_unknown_ratio"]),
        "fusion_consistency_mean": stat(data, ["fusion_consistency", "consistency_score"]),
        "fusion_calibration_drift_mean": stat(data, ["fusion_consistency", "calibration_drift_score"]),
        "fusion_projection_error_px_mean": stat(data, ["fusion_consistency", "mean_nearest_camera_error_px"]),
        "fusion_stamp_delta_ms_mean": stat(data, ["fusion_consistency", "abs_camera_lidar_stamp_delta_ms"]),
        "runtime_budget_states": compact_counts(get_path(data, ["runtime_budget", "state_counts"])),
        "runtime_budget_total_p95_mean_ms": stat(data, ["runtime_budget", "runtime_budget_total_p95_ms"]),
        "task_risk_states": compact_counts(get_path(data, ["task_risk", "state_counts"])),
        "task_risk_score_mean": stat(data, ["task_risk", "task_risk_score"]),
        "last_write_policy": get_path(data, ["task_risk", "last_write_policy"]),
        "planning_frames": get_path(data, ["planning", "frames"]),
        "planning_ready_frames": ready_count(data),
    }

    planning_frames = number(row.get("planning_frames"))
    planning_ready = number(row.get("planning_ready_frames"))
    row["planning_ready_ratio"] = (
        None if not planning_frames or planning_ready is None else planning_ready / planning_frames
    )
    for field, topic in TOPICS.items():
        row[field] = topic_rate(data, topic)
    return {field: row.get(field) for field in SUMMARY_FIELDS}


def write_csv(path: Path, rows: List[Dict[str, Any]], fields: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(fields))
        writer.writeheader()
        writer.writerows(rows)


def mean_of(rows: Iterable[Dict[str, Any]], field: str) -> Optional[float]:
    values = [number(row.get(field)) for row in rows]
    values = [value for value in values if value is not None]
    if not values:
        return None
    return fmean(values)


def group_value(value: Any) -> str:
    if value is None or value == "":
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def aggregate_rows(rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    groups: Dict[tuple, List[Dict[str, Any]]] = defaultdict(list)
    for row in rows:
        key = (
            group_value(row.get("run_type")),
            group_value(row.get("profile")),
            group_value(row.get("backend_effective")),
            group_value(row.get("mapping_gate")),
            group_value(row.get("lidar_verifier")),
        )
        groups[key].append(row)

    out: List[Dict[str, Any]] = []
    for (run_type, profile, backend, gate, verifier), group in sorted(groups.items()):
        out.append(
            {
                "run_type": run_type,
                "profile": profile,
                "backend_effective": backend,
                "mapping_gate": gate,
                "lidar_verifier": verifier,
                "runs": len(group),
                "successes": sum(1 for row in group if str(row.get("success")).lower() == "true"),
                "global_map_hz_mean": mean_of(group, "global_map_hz"),
                "lidar_cones_hz_mean": mean_of(group, "lidar_cones_hz"),
                "pointpillars_total_mean_ms": mean_of(group, "pointpillars_total_mean_ms"),
                "cluster_total_mean_ms": mean_of(group, "cluster_total_mean_ms"),
                "final_stable_cones_mean": mean_of(group, "final_stable_cones"),
                "candidate_residue_total_mean": mean_of(group, "candidate_residue_total"),
                "map_stability_score_mean": mean_of(group, "map_stability_score"),
                "fusion_consistency_mean": mean_of(group, "fusion_consistency_mean"),
                "fusion_calibration_drift_mean": mean_of(group, "fusion_calibration_drift_mean"),
                "planning_ready_ratio_mean": mean_of(group, "planning_ready_ratio"),
            }
        )
    return out


def load_gate_comparisons(paths: Iterable[Path]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for path in paths:
        with path.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                row = dict(row)
                row["source_csv"] = str(path)
                rows.append(row)
    return rows


def row_line(row: Dict[str, Any], fields: Sequence[str]) -> str:
    return "| " + " | ".join(fmt(row.get(field)) for field in fields) + " |"


def markdown_table(rows: Sequence[Dict[str, Any]], fields: Sequence[str], headers: Sequence[str]) -> List[str]:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    lines.extend(row_line(row, fields) for row in rows)
    return lines


def latest_rows(rows: List[Dict[str, Any]], run_type: str, limit: int = 20) -> List[Dict[str, Any]]:
    filtered = [row for row in rows if row.get("run_type") == run_type]
    return filtered[:limit]


def generate_markdown(
    rows: List[Dict[str, Any]],
    aggregates: List[Dict[str, Any]],
    gate_rows: List[Dict[str, Any]],
    output_dir: Path,
    max_runs: int,
) -> str:
    lines: List[str] = [
        "# RacingBrain Benchmark Tables",
        "",
        f"Generated: `{datetime.now().isoformat(timespec='seconds')}`",
        f"Runs summarized: `{len(rows)}`",
        f"Output directory: `{output_dir}`",
        "",
        "This file is generated by `scripts/eval/summarize_benchmarks.py`.",
        "It is intended for paper drafts, advisor updates, and quick regression checks.",
        "",
        "## Headline Replay Runs",
        "",
    ]

    headline_fields = [
        "run_name",
        "profile",
        "backend_effective",
        "mapping_gate",
        "global_map_hz",
        "lidar_cones_hz",
        "final_stable_cones",
        "candidate_residue_total",
        "map_stability_score",
        "fusion_consistency_mean",
        "fusion_calibration_drift_mean",
        "task_risk_states",
    ]
    lines.extend(
        markdown_table(
            latest_rows(rows, "mapping_replay", max_runs),
            headline_fields,
            [
                "Run",
                "Profile",
                "Backend",
                "Gate",
                "Global Hz",
                "LiDAR Hz",
                "Stable",
                "Residue",
                "Stability",
                "Consistency",
                "Drift",
                "Task Risk",
            ],
        )
    )

    camera_rows = latest_rows(rows, "camera_detector_benchmark", 12)
    if camera_rows:
        lines.extend(["", "## Camera Detector Timing", ""])
        lines.extend(
            markdown_table(
                camera_rows,
                [
                    "run_name",
                    "camera_yolo_gpu_mean_ms",
                    "camera_yolo_gpu_p95_ms",
                    "camera_classical_cone_mean_ms",
                    "camera_classical_cone_p95_ms",
                    "camera_yolo_boxes_mean",
                    "camera_classical_candidates_mean",
                ],
                [
                    "Run",
                    "YOLO Mean ms",
                    "YOLO P95 ms",
                    "Classical Mean ms",
                    "Classical P95 ms",
                    "YOLO Boxes",
                    "Classical Candidates",
                ],
            )
        )

    lines.extend(["", "## Aggregate View", ""])
    lines.extend(
        markdown_table(
            aggregates,
            [
                "run_type",
                "profile",
                "backend_effective",
                "mapping_gate",
                "lidar_verifier",
                "runs",
                "successes",
                "global_map_hz_mean",
                "pointpillars_total_mean_ms",
                "cluster_total_mean_ms",
                "map_stability_score_mean",
                "planning_ready_ratio_mean",
            ],
            [
                "Type",
                "Profile",
                "Backend",
                "Gate",
                "Verifier",
                "Runs",
                "OK",
                "Global Hz",
                "PP ms",
                "Cluster ms",
                "Stability",
                "Planning Ready",
            ],
        )
    )

    if gate_rows:
        gate_fields = list(gate_rows[0].keys())
        selected_fields = [
            field
            for field in [
                "scenario",
                "stable_cones_delta_on_minus_off",
                "duplicate_pairs_delta_on_minus_off",
                "candidate_residue_delta_on_minus_off",
                "stability_score_delta_on_minus_off",
                "downweighted_observations_gate_on",
                "rejected_new_cones_gate_on",
                "source_csv",
            ]
            if field in gate_fields
        ]
        lines.extend(["", "## Mapping Gate Comparisons", ""])
        lines.extend(markdown_table(gate_rows[:40], selected_fields, selected_fields))

    lines.extend(
        [
            "",
            "## Reading Notes",
            "",
            "- `map_stability_score` is a self-consistency metric, not absolute map accuracy.",
            "- `candidate_residue_total` and `final_duplicate_density` are map-pollution indicators.",
            "- `fusion_calibration_drift_mean` is a suspicion score from projection residual, low IoU, forced matching, and timing.",
            "- Use annotated cone positions or surveyed track geometry before making absolute accuracy claims.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    input_roots = args.input_root or [Path("log/eval")]
    benchmark_roots = args.benchmark_root or [Path("log/benchmark")]

    summaries = discover_summaries(input_roots)
    rows = []
    for path in summaries:
        try:
            rows.append(collect_row(path))
        except Exception as exc:  # keep summary generation robust across old logs
            rows.append(
                {
                    **{field: None for field in SUMMARY_FIELDS},
                    "run_name": path.parent.name,
                    "run_type": "parse_error",
                    "run_dir": str(path.parent),
                    "success": False,
                    "profile": f"parse_error:{exc}",
                }
            )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    aggregate = aggregate_rows(rows)
    gate_rows = load_gate_comparisons(discover_gate_comparisons(benchmark_roots))

    write_csv(args.output_dir / "runs_summary.csv", rows, SUMMARY_FIELDS)
    write_csv(args.output_dir / "aggregate_summary.csv", aggregate, list(aggregate[0].keys()) if aggregate else [])
    if gate_rows:
        write_csv(args.output_dir / "gate_comparisons.csv", gate_rows, list(gate_rows[0].keys()))

    markdown = generate_markdown(rows, aggregate, gate_rows, args.output_dir, args.max_markdown_runs)
    (args.output_dir / "paper_tables.md").write_text(markdown + "\n", encoding="utf-8")
    manifest = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "input_roots": [str(path) for path in input_roots],
        "benchmark_roots": [str(path) for path in benchmark_roots],
        "summary_count": len(summaries),
        "gate_comparison_count": len(gate_rows),
        "outputs": [
            "runs_summary.csv",
            "aggregate_summary.csv",
            "gate_comparisons.csv" if gate_rows else None,
            "paper_tables.md",
        ],
    }
    (args.output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    print(f"Summarized {len(rows)} runs into {args.output_dir}")
    print(f"- {args.output_dir / 'runs_summary.csv'}")
    print(f"- {args.output_dir / 'aggregate_summary.csv'}")
    print(f"- {args.output_dir / 'paper_tables.md'}")
    if gate_rows:
        print(f"- {args.output_dir / 'gate_comparisons.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
