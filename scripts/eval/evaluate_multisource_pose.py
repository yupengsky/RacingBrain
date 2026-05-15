#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Tuple


REPO_ROOT = Path(__file__).resolve().parents[2]
RACINGBRAIN_PATH = REPO_ROOT / "LocalizationMapping" / "RacingBrain"
if str(RACINGBRAIN_PATH) not in sys.path:
    sys.path.insert(0, str(RACINGBRAIN_PATH))

from racingbrain.localization.pose_sources import MultiSourcePoseJudge, Pose2D, PoseJudgeConfig, wrap_angle


def true_pose(t: float) -> Pose2D:
    x = 5.0 * t
    y = 1.2 * math.sin(0.32 * t)
    dy_dt = 1.2 * 0.32 * math.cos(0.32 * t)
    yaw = math.atan2(dy_dt, 5.0)
    return Pose2D(stamp=t, x=x, y=y, yaw=yaw, vx=5.0, vy=dy_dt, yaw_rate=0.0, source="truth")


def transform_truth_to_lio_raw(pose: Pose2D, yaw_offset: float, tx: float, ty: float) -> Pose2D:
    # Inverse of map = R(yaw_offset) * lio + t.
    dx = pose.x - tx
    dy = pose.y - ty
    c = math.cos(-yaw_offset)
    s = math.sin(-yaw_offset)
    return Pose2D(
        stamp=pose.stamp,
        x=c * dx - s * dy,
        y=s * dx + c * dy,
        yaw=wrap_angle(pose.yaw - yaw_offset),
        vx=pose.vx,
        vy=pose.vy,
        yaw_rate=pose.yaw_rate,
        frame_id="odom",
        source="lio_raw",
    )


def noisy(value: float, sigma: float) -> float:
    return value + random.gauss(0.0, sigma)


def scenario_name(t: float) -> str:
    if 10.0 <= t < 15.0:
        return "gnss_accuracy_degraded"
    if 22.0 <= t < 28.0:
        return "gnss_dropout"
    if 35.0 <= t < 42.0:
        return "lio_covariance_degraded"
    return "nominal"


def generate_synthetic_trace(duration: float, dt: float, seed: int) -> Iterable[Dict[str, object]]:
    random.seed(seed)
    yaw_offset = 0.14
    tx = 2.5
    ty = -1.2
    lio_drift_x = 0.0
    lio_drift_y = 0.0
    steps = int(duration / dt)
    for index in range(steps + 1):
        t = round(index * dt, 6)
        truth = true_pose(t)
        scenario = scenario_name(t)

        gnss_available = scenario != "gnss_dropout"
        gnss_accuracy = 0.18
        gnss_noise = 0.07
        gnss_bias_x = 0.0
        gnss_bias_y = 0.0
        if scenario == "gnss_accuracy_degraded":
            gnss_accuracy = 3.2
            gnss_noise = 0.30
            gnss_bias_x = 2.8
            gnss_bias_y = -1.0

        lio_covariance = 0.04
        if scenario == "lio_covariance_degraded":
            lio_covariance = 1.2
            lio_drift_x += 0.035
            lio_drift_y -= 0.018
        else:
            lio_drift_x += 0.004
            lio_drift_y += 0.001

        gnss_pose = None
        if gnss_available:
            gnss_pose = Pose2D(
                stamp=t,
                x=noisy(truth.x + gnss_bias_x, gnss_noise),
                y=noisy(truth.y + gnss_bias_y, gnss_noise),
                yaw=wrap_angle(noisy(truth.yaw, 0.012)),
                vx=truth.vx,
                vy=truth.vy,
                yaw_rate=truth.yaw_rate,
                source="gnss_ins",
            )

        lio_truth = Pose2D(
            stamp=t,
            x=truth.x + lio_drift_x,
            y=truth.y + lio_drift_y,
            yaw=wrap_angle(truth.yaw + random.gauss(0.0, 0.010)),
            vx=truth.vx,
            vy=truth.vy,
            yaw_rate=truth.yaw_rate,
            source="lio_map_equiv",
        )
        lio_pose = transform_truth_to_lio_raw(lio_truth, yaw_offset, tx, ty)

        yield {
            "t": t,
            "scenario": scenario,
            "truth": truth,
            "gnss": gnss_pose,
            "gnss_accuracy": gnss_accuracy,
            "lio": lio_pose,
            "lio_covariance": lio_covariance,
        }


def run_synthetic(args: argparse.Namespace) -> Tuple[List[Dict[str, object]], Dict[str, object]]:
    judge = MultiSourcePoseJudge(
        PoseJudgeConfig(
            stale_timeout_sec=args.stale_timeout_sec,
            fusion_enabled=not args.disable_fusion,
            cross_position_warn_m=args.cross_position_warn_m,
            cross_position_reject_m=args.cross_position_reject_m,
        )
    )
    rows: List[Dict[str, object]] = []
    by_scenario: Dict[str, List[float]] = defaultdict(list)
    source_counts: Counter[str] = Counter()
    state_counts: Counter[str] = Counter()

    for frame in generate_synthetic_trace(args.duration_sec, args.dt, args.seed):
        t = float(frame["t"])
        truth: Pose2D = frame["truth"]  # type: ignore[assignment]
        gnss_pose = frame["gnss"]
        if gnss_pose is not None:
            judge.update_gnss(gnss_pose, accuracy_xy=float(frame["gnss_accuracy"]))  # type: ignore[arg-type]
        judge.update_lio(frame["lio"], covariance_xy=float(frame["lio_covariance"]))  # type: ignore[arg-type]
        decision = judge.decide(t)

        error = None
        if decision.pose is not None:
            error = decision.pose.distance_to(truth)
            by_scenario[str(frame["scenario"])].append(error)
        source_counts[decision.source] += 1
        state_counts[decision.state] += 1
        rows.append(
            {
                "t": t,
                "scenario": frame["scenario"],
                "decision_state": decision.state,
                "decision_source": decision.source,
                "error_m": "" if error is None else round(error, 4),
                "cross_position_error_m": ""
                if decision.cross_position_error_m is None
                else round(decision.cross_position_error_m, 4),
                "cross_yaw_error_rad": ""
                if decision.cross_yaw_error_rad is None
                else round(decision.cross_yaw_error_rad, 5),
                "gnss_score": round(decision.qualities["gnss_ins"].score, 4),
                "lio_score": round(decision.qualities["lio"].score, 4),
                "reasons": ";".join(decision.reasons),
            }
        )

    def mean(values: List[float]) -> float:
        return sum(values) / len(values) if values else float("nan")

    summary = {
        "duration_sec": args.duration_sec,
        "dt": args.dt,
        "seed": args.seed,
        "source_counts": dict(source_counts),
        "state_counts": dict(state_counts),
        "mean_error_m_by_scenario": {key: round(mean(values), 4) for key, values in sorted(by_scenario.items())},
        "max_error_m_by_scenario": {key: round(max(values), 4) for key, values in sorted(by_scenario.items()) if values},
    }
    return rows, summary


def write_outputs(rows: List[Dict[str, object]], summary: Dict[str, object], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "decision_trace.csv"
    if rows:
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = [
        "# Multi-source Pose Judge Synthetic Evaluation",
        "",
        f"- Duration: `{summary['duration_sec']}` s",
        f"- Step: `{summary['dt']}` s",
        f"- Source counts: `{summary['source_counts']}`",
        f"- State counts: `{summary['state_counts']}`",
        f"- Mean error by scenario: `{summary['mean_error_m_by_scenario']}`",
        f"- Max error by scenario: `{summary['max_error_m_by_scenario']}`",
    ]
    (output_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate GNSS/INS + LIO pose-source judgement.")
    parser.add_argument("--output-dir", default="log/benchmark/multisource_pose/latest")
    parser.add_argument("--duration-sec", type=float, default=50.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--stale-timeout-sec", type=float, default=0.35)
    parser.add_argument("--cross-position-warn-m", type=float, default=0.75)
    parser.add_argument("--cross-position-reject-m", type=float, default=2.0)
    parser.add_argument("--disable-fusion", action="store_true")
    args = parser.parse_args()

    rows, summary = run_synthetic(args)
    output_dir = Path(args.output_dir)
    write_outputs(rows, summary, output_dir)
    print(json.dumps({"output_dir": str(output_dir), **summary}, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
