#!/usr/bin/env python3
from __future__ import annotations

import argparse
import bisect
import csv
import html
import json
import math
import re
import sqlite3
import statistics
import time
from collections import Counter, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Dict, List, Optional, Sequence, Tuple

from offline_simple_lio_eval import (
    IcpResult,
    Keyframe,
    Pose2,
    SimpleLioConfig,
    build_local_map,
    clamp_motion,
    extract_scan_2d,
    finite_stats,
    iter_cloud_rows,
    make_gnss_interpolator,
    open_bag_db,
    predict_pose,
    read_gnss_truth,
    read_imu_series,
    round_or_none,
    run_icp,
    should_add_keyframe,
    status_counts,
    stamp_to_sec,
    topic_ids,
    topic_metadata,
    transform_points,
    voxel_downsample,
)
from racingbrain.localization.pose_sources import wrap_angle


DEFAULT_OUTAGES = "45:8,185:10,360:8"
SEVERITY_COUNTS = (1, 5, 10)
SEVERITY_DURATIONS_SEC = (5.0, 10.0)


@dataclass(frozen=True)
class RelativeOutageSpec:
    outage_id: str
    start_sec: float
    duration_sec: float


@dataclass(frozen=True)
class OutageWindow:
    outage_id: str
    start_stamp: float
    end_stamp: float
    start_t: float
    end_t: float
    duration_sec: float

    def contains(self, stamp: float) -> bool:
        return self.start_stamp <= stamp < self.end_stamp


@dataclass
class OutageLioRuntime:
    window: OutageWindow
    anchor_pose: Pose2
    keyframes: Deque[Keyframe]
    previous_pose: Optional[Pose2] = None
    previous_previous_pose: Optional[Pose2] = None
    processed_samples: int = 0
    icp_used_count: int = 0
    sparse_prediction_count: int = 0


@dataclass(frozen=True)
class ScenarioSpec:
    scenario_id: str
    label: str
    outage_count: int
    segment_duration_sec: float
    outage_specs: List[RelativeOutageSpec]


@dataclass
class ScenarioState:
    spec: ScenarioSpec
    windows: List[OutageWindow]
    active_lio: Optional[OutageLioRuntime] = None
    outage_initializations: List[Dict[str, object]] = None
    outage_runtime_by_id: Dict[str, OutageLioRuntime] = None
    samples: List[Dict[str, object]] = None
    error_sum: float = 0.0
    fallback_scan_points: List[int] = None
    fallback_icp_used: int = 0

    def __post_init__(self) -> None:
        if self.outage_initializations is None:
            self.outage_initializations = []
        if self.outage_runtime_by_id is None:
            self.outage_runtime_by_id = {}
        if self.samples is None:
            self.samples = []
        if self.fallback_scan_points is None:
            self.fallback_scan_points = []


def parse_outage_specs(spec_text: str) -> List[RelativeOutageSpec]:
    specs: List[RelativeOutageSpec] = []
    for raw in re.split(r"[,;\s]+", spec_text.strip()):
        if not raw:
            continue
        label: Optional[str] = None
        body = raw
        if "=" in raw:
            label, body = raw.split("=", 1)
        elif "@" in raw:
            label, body = raw.split("@", 1)
        parts = re.split(r"[:+]", body)
        if len(parts) != 2:
            raise ValueError(f"Invalid outage spec '{raw}'. Use start:duration or gap_id=start:duration.")
        start_sec = float(parts[0])
        duration_sec = float(parts[1])
        if start_sec < 0.0 or duration_sec <= 0.0:
            raise ValueError(f"Invalid outage spec '{raw}'. Start must be >= 0 and duration must be > 0.")
        outage_id = (label or f"gap_{len(specs) + 1:02d}").strip()
        if not outage_id:
            outage_id = f"gap_{len(specs) + 1:02d}"
        specs.append(RelativeOutageSpec(outage_id=outage_id, start_sec=start_sec, duration_sec=duration_sec))

    specs.sort(key=lambda item: item.start_sec)
    for prev, cur in zip(specs, specs[1:]):
        if cur.start_sec < prev.start_sec + prev.duration_sec:
            raise ValueError(
                f"Outage windows overlap: {prev.outage_id} and {cur.outage_id}. "
                "Use non-overlapping artificial GNSS/INS outages."
            )
    return specs


def make_even_outage_specs(outage_count: int, segment_duration_sec: float, total_duration_sec: float) -> List[RelativeOutageSpec]:
    if outage_count <= 0:
        return []
    duration = float(segment_duration_sec)
    total = max(duration + 1.0, float(total_duration_sec))
    margin = min(55.0, max(30.0, total * 0.08))
    usable_start = margin
    usable_end = max(usable_start + duration, total - margin)
    if outage_count == 1:
        starts = [max(0.0, total * 0.5 - duration * 0.5)]
    else:
        first_center = usable_start + duration * 0.5
        last_center = usable_end - duration * 0.5
        if last_center <= first_center:
            spacing = duration * 1.5
            starts = [usable_start + i * spacing for i in range(outage_count)]
        else:
            starts = [
                first_center + (last_center - first_center) * i / max(1, outage_count - 1) - duration * 0.5
                for i in range(outage_count)
            ]

    max_start = max(0.0, total - duration - 0.5)
    specs: List[RelativeOutageSpec] = []
    previous_end = -math.inf
    for index, raw_start in enumerate(starts):
        start = max(0.0, min(max_start, raw_start))
        if start < previous_end:
            start = min(max_start, previous_end)
        previous_end = start + duration
        specs.append(
            RelativeOutageSpec(
                outage_id=f"gap_{index + 1:02d}",
                start_sec=round(start, 6),
                duration_sec=duration,
            )
        )
    return specs


def build_scenario_specs(args: argparse.Namespace, total_duration_sec: float) -> List[ScenarioSpec]:
    if args.scenario_mode == "single":
        outage_specs = parse_outage_specs(args.outages)
        total_gap = sum(spec.duration_sec for spec in outage_specs)
        return [
            ScenarioSpec(
                scenario_id="custom",
                label=f"Custom ({len(outage_specs)} gaps, {total_gap:g}s total)",
                outage_count=len(outage_specs),
                segment_duration_sec=0.0,
                outage_specs=outage_specs,
            )
        ]

    if (args.gap_count is None) != (args.gap_duration_sec is None):
        raise ValueError("--gap-count and --gap-duration-sec must be provided together.")

    if args.gap_count is not None and args.gap_duration_sec is not None:
        count = int(args.gap_count)
        duration = float(args.gap_duration_sec)
        if count <= 0 or duration <= 0.0:
            raise ValueError("--gap-count and --gap-duration-sec must both be positive.")
        specs = make_even_outage_specs(count, duration, total_duration_sec)
        return [
            ScenarioSpec(
                scenario_id=f"{count}_gaps_{duration:g}s".replace(".", "p"),
                label=f"{count} gap{'s' if count != 1 else ''} x {duration:g}s",
                outage_count=count,
                segment_duration_sec=duration,
                outage_specs=specs,
            )
        ]

    scenarios: List[ScenarioSpec] = []
    for count in SEVERITY_COUNTS:
        for duration in SEVERITY_DURATIONS_SEC:
            specs = make_even_outage_specs(count, duration, total_duration_sec)
            scenarios.append(
                ScenarioSpec(
                    scenario_id=f"{count}_gaps_{int(duration)}s",
                    label=f"{count} gap{'s' if count != 1 else ''} x {duration:g}s",
                    outage_count=count,
                    segment_duration_sec=duration,
                    outage_specs=specs,
                )
            )
    return scenarios


def build_outage_windows(first_cloud_stamp: float, specs: Sequence[RelativeOutageSpec]) -> List[OutageWindow]:
    windows = []
    for spec in specs:
        start_stamp = first_cloud_stamp + spec.start_sec
        end_stamp = start_stamp + spec.duration_sec
        windows.append(
            OutageWindow(
                outage_id=spec.outage_id,
                start_stamp=start_stamp,
                end_stamp=end_stamp,
                start_t=spec.start_sec,
                end_t=spec.start_sec + spec.duration_sec,
                duration_sec=spec.duration_sec,
            )
        )
    return windows


def outage_at(stamp: float, windows: Sequence[OutageWindow]) -> Optional[OutageWindow]:
    for window in windows:
        if window.contains(stamp):
            return window
    return None


def pose_in_any_outage(stamp: float, windows: Sequence[OutageWindow]) -> bool:
    return outage_at(stamp, windows) is not None


def latest_available_gnss_pose_before(truth_poses: Sequence[Pose2], stamp: float, windows: Sequence[OutageWindow]) -> Pose2:
    stamps = [pose.stamp for pose in truth_poses]
    index = bisect.bisect_left(stamps, stamp) - 1
    while index >= 0:
        pose = truth_poses[index]
        if not pose_in_any_outage(pose.stamp, windows):
            return pose
        index -= 1
    raise RuntimeError(f"No available GNSS/INS pose exists before outage start {stamp:.6f}")


def cfg_from_args(args: argparse.Namespace) -> SimpleLioConfig:
    return SimpleLioConfig(
        lidar_min_range_m=args.lidar_min_range,
        lidar_max_range_m=args.lidar_max_range,
        z_min_m=args.z_min,
        z_max_m=args.z_max,
        icp_z_weight=args.icp_z_weight,
        scan_voxel_m=args.scan_voxel,
        map_voxel_m=args.map_voxel,
        max_scan_points=args.max_scan_points,
        max_keyframe_points=args.max_keyframe_points,
        local_map_keyframes=args.local_map_keyframes,
        keyframe_dist_m=args.keyframe_dist,
        keyframe_yaw_rad=args.keyframe_yaw,
        keyframe_time_sec=args.keyframe_time,
        icp_iterations=args.icp_iterations,
        icp_max_correspondence_m=args.icp_max_correspondence,
        icp_trim_fraction=args.icp_trim_fraction,
        icp_min_inliers=args.icp_min_inliers,
        icp_max_rmse_m=args.icp_max_rmse,
        icp_max_translation_step_m=args.icp_max_translation_step,
        icp_max_yaw_step_rad=args.icp_max_yaw_step,
        icp_max_pose_correction_m=args.icp_max_pose_correction,
        icp_max_pose_correction_yaw_rad=args.icp_max_pose_correction_yaw,
        imu_gyro_scale=args.imu_gyro_scale,
        max_frame_dt_sec=args.max_frame_dt,
        max_speed_mps=args.max_speed,
        max_yaw_rate_radps=args.max_yaw_rate,
        sparse_scan_points=args.sparse_scan_points,
        sparse_icp_weight=args.sparse_icp_weight,
        velocity_update_alpha=args.velocity_update_alpha,
        sparse_velocity_update_alpha=args.sparse_velocity_update_alpha,
    )


def run_outage_lio_step(
    runtime: OutageLioRuntime,
    *,
    stamp: float,
    scan,
    imu,
    cfg: SimpleLioConfig,
) -> Tuple[Pose2, IcpResult]:
    if runtime.previous_pose is None:
        pose = predict_pose(runtime.anchor_pose, None, stamp, imu, cfg)
        icp = IcpResult(
            pose=pose,
            used=False,
            inliers=0,
            rmse=None,
            median_error=None,
            iterations=0,
            reason="outage_initialized_from_latest_available_gnss_ins",
        )
    else:
        predicted = predict_pose(runtime.previous_pose, runtime.previous_previous_pose, stamp, imu, cfg)
        if len(scan) < cfg.icp_min_inliers:
            pose = predicted
            runtime.sparse_prediction_count += 1
            icp = IcpResult(
                pose=pose,
                used=False,
                inliers=0,
                rmse=None,
                median_error=None,
                iterations=0,
                reason="scan_too_sparse_prediction_only",
            )
        else:
            local_map = build_local_map(runtime.keyframes, predicted, cfg)
            icp = run_icp(scan, local_map, predicted, cfg)
            pose = icp.pose
            if icp.used:
                runtime.icp_used_count += 1
            if icp.used and len(scan) < cfg.sparse_scan_points:
                pose = blend_pose(predicted, pose, cfg.sparse_icp_weight)
                icp.pose = pose
                icp.reason = f"{icp.reason}_sparse_scan_blend"

    if runtime.previous_pose is not None:
        pose, motion_clamped = clamp_motion(runtime.previous_pose, pose, cfg)
        if motion_clamped:
            icp.reason = f"{icp.reason}_motion_clamped"
            icp.pose = pose
        dt = pose.stamp - runtime.previous_pose.stamp
        if 1e-3 < dt <= cfg.max_frame_dt_sec:
            observed_vx = (pose.x - runtime.previous_pose.x) / dt
            observed_vy = (pose.y - runtime.previous_pose.y) / dt
            alpha = cfg.sparse_velocity_update_alpha if len(scan) < cfg.sparse_scan_points else cfg.velocity_update_alpha
            pose.vx = runtime.previous_pose.vx * (1.0 - alpha) + observed_vx * alpha
            pose.vy = runtime.previous_pose.vy * (1.0 - alpha) + observed_vy * alpha
        else:
            pose.vx = runtime.previous_pose.vx
            pose.vy = runtime.previous_pose.vy

    if len(scan) >= cfg.icp_min_inliers and should_add_keyframe(runtime.keyframes, pose, cfg):
        world_points = voxel_downsample(transform_points(scan, pose), cfg.map_voxel_m, cfg.max_keyframe_points)
        runtime.keyframes.append(
            Keyframe(stamp=pose.stamp, x=pose.x, y=pose.y, yaw=pose.yaw, points_world=world_points)
        )
        while len(runtime.keyframes) > cfg.local_map_keyframes:
            runtime.keyframes.popleft()

    runtime.previous_previous_pose = runtime.previous_pose
    runtime.previous_pose = pose
    runtime.processed_samples += 1
    return pose, icp


def blend_pose(a: Pose2, b: Pose2, weight_b: float) -> Pose2:
    weight = max(0.0, min(1.0, float(weight_b)))
    return Pose2(
        stamp=b.stamp,
        x=a.x + (b.x - a.x) * weight,
        y=a.y + (b.y - a.y) * weight,
        yaw=wrap_angle(a.yaw + wrap_angle(b.yaw - a.yaw) * weight),
        vx=a.vx,
        vy=a.vy,
    )


def mean_so_far(total: float, count: int) -> float:
    return total / max(1, count)


def finite_rmse(values: Sequence[float]) -> Optional[float]:
    clean = [float(value) for value in values if math.isfinite(float(value))]
    if not clean:
        return None
    return round(math.sqrt(statistics.fmean([value * value for value in clean])), 6)


def make_fusion_sample(
    *,
    scenario_id: str,
    scenario_label: str,
    elapsed: float,
    cloud_stamp: float,
    truth_pose: Pose2,
    fused_pose: Pose2,
    fused_source: str,
    gnss_available: bool,
    outage_id: str,
    icp: IcpResult,
    scan_points: Optional[int],
    keyframes: Optional[int],
    error_sum: float,
    sample_count: int,
) -> Dict[str, object]:
    position_error = math.hypot(fused_pose.x - truth_pose.x, fused_pose.y - truth_pose.y)
    yaw_error = abs(wrap_angle(fused_pose.yaw - truth_pose.yaw))
    return {
        "scenario_id": scenario_id,
        "scenario_label": scenario_label,
        "t": round(elapsed, 6),
        "stamp": round(cloud_stamp, 6),
        "truth": {
            "x": round(truth_pose.x, 4),
            "y": round(truth_pose.y, 4),
            "yaw": round(truth_pose.yaw, 6),
        },
        "fused": {
            "x": round(fused_pose.x, 4),
            "y": round(fused_pose.y, 4),
            "yaw": round(fused_pose.yaw, 6),
            "source": fused_source,
        },
        "gnss_available": bool(gnss_available),
        "outage_id": outage_id,
        "error_m": round(position_error, 4),
        "mean_error_m": round(mean_so_far(error_sum + position_error, sample_count + 1), 4),
        "yaw_error_rad": round(yaw_error, 6),
        "icp": {
            "used": bool(icp.used),
            "inliers": int(icp.inliers),
            "rmse_m": round_or_none(icp.rmse, 4),
            "median_error_m": round_or_none(icp.median_error, 4),
            "iterations": int(icp.iterations),
            "reason": icp.reason,
        },
        "scan_points": scan_points,
        "keyframes": keyframes,
    }


def summarize_scenario(
    state: ScenarioState,
    *,
    processed_clouds: int,
    skipped_clouds: int,
    truth,
    cfg: SimpleLioConfig,
    runtime_sec: float,
) -> Dict[str, object]:
    samples = state.samples
    errors = [float(sample["error_m"]) for sample in samples]
    yaw_errors = [float(sample["yaw_error_rad"]) for sample in samples]
    fallback_samples = [sample for sample in samples if sample["fused"]["source"] == "lio_fallback"]
    fallback_errors = [float(sample["error_m"]) for sample in fallback_samples]
    fallback_yaw_errors = [float(sample["yaw_error_rad"]) for sample in fallback_samples]
    available_errors = [float(sample["error_m"]) for sample in samples if sample["gnss_available"]]
    source_counts = Counter(str(sample["fused"]["source"]) for sample in samples)

    outage_summaries = []
    for window in state.windows:
        window_samples = [sample for sample in samples if sample["outage_id"] == window.outage_id]
        window_errors = [float(sample["error_m"]) for sample in window_samples]
        runtime = state.outage_runtime_by_id.get(window.outage_id)
        outage_summaries.append(
            {
                "outage_id": window.outage_id,
                "start_t": round(window.start_t, 6),
                "end_t": round(window.end_t, 6),
                "duration_sec": round(window.duration_sec, 6),
                "fallback_sample_count": len(window_samples),
                "position_error_m": finite_stats(window_errors),
                "lio_processed_samples": 0 if runtime is None else runtime.processed_samples,
                "lio_icp_used_count": 0 if runtime is None else runtime.icp_used_count,
                "lio_sparse_prediction_count": 0 if runtime is None else runtime.sparse_prediction_count,
            }
        )

    outage_total_duration = sum(window.duration_sec for window in state.windows)
    duration = float(samples[-1]["t"]) if samples else 0.0
    return {
        "scenario_id": state.spec.scenario_id,
        "scenario_label": state.spec.label,
        "segment_duration_sec": state.spec.segment_duration_sec,
        "sample_count": len(samples),
        "duration_sec": round(duration, 6),
        "processed_clouds": processed_clouds,
        "skipped_clouds": skipped_clouds,
        "source_counts": {key: int(value) for key, value in sorted(source_counts.items())},
        "outage_count": len(state.windows),
        "outage_total_duration_sec": round(outage_total_duration, 6),
        "fallback_sample_count": len(fallback_samples),
        "fallback_icp_used_count": int(state.fallback_icp_used),
        "fallback_icp_used_ratio": round(state.fallback_icp_used / max(1, len(fallback_samples) - len(state.windows)), 6),
        "position_error_m": finite_stats(errors),
        "fallback_position_error_m": finite_stats(fallback_errors),
        "available_position_error_m": finite_stats(available_errors),
        "yaw_error_rad": finite_stats(yaw_errors),
        "fallback_yaw_error_rad": finite_stats(fallback_yaw_errors),
        "fallback_scan_points": finite_stats(state.fallback_scan_points),
        "fallback_rmse_m": finite_rmse(fallback_errors),
        "gnss_accuracy_horizon_m": finite_stats(truth.accuracy_horizon_m),
        "gnss_accuracy_yaw_deg": finite_stats(truth.accuracy_yaw_deg),
        "gnss_ins_status_counts": status_counts(truth.ins_status),
        "gnss_status_counts": status_counts(truth.gnss_status),
        "satellite_main": finite_stats(truth.satellite_main),
        "satellite_sub": finite_stats(truth.satellite_sub),
        "outages": outage_summaries,
        "outage_initializations": state.outage_initializations,
        "method": {
            "name": "gnss_ins_priority_with_simple_lio_outage_fallback",
            "truth_definition": "The complete unmasked GNSS/INS trajectory is the ground-truth trajectory for all error metrics.",
            "fusion_rule": "Fused pose equals GNSS/INS outside artificial outages and equals simple LIO fallback inside artificial outages.",
            "fallback_rule": "Each outage starts a fresh simple LIO runtime initialized from the latest GNSS/INS pose before the outage; GNSS/INS truth is not injected during the outage.",
            "lio_algorithm": "Reuses offline_simple_lio_eval.py scan filtering, IMU yaw prediction, local keyframe map, and SE(2) ICP parameters.",
        },
        "config": cfg.__dict__,
        "runtime_sec": round(runtime_sec, 3),
    }


def evaluate_dataset(args: argparse.Namespace) -> Dict[str, object]:
    dataset_dir = Path(args.dataset_dir).expanduser().resolve()
    db_path = open_bag_db(dataset_dir)
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    cfg = cfg_from_args(args)
    started = time.time()

    conn = sqlite3.connect(str(db_path))
    ids = topic_ids(conn)
    for topic in (args.cloud_topic, args.imu_topic, args.gnss_topic):
        if topic not in ids:
            raise RuntimeError(f"Required topic is missing from bag: {topic}")

    metadata = topic_metadata(conn)
    truth = read_gnss_truth(conn, ids[args.gnss_topic])
    imu = read_imu_series(conn, ids[args.imu_topic], cfg.imu_gyro_scale)
    interp_truth = make_gnss_interpolator(truth)

    start_stamp = truth.poses[0].stamp + max(0.0, args.start_sec)
    end_stamp = None if args.duration_sec <= 0.0 else start_stamp + args.duration_sec

    first_cloud_stamp: Optional[float] = None
    states: List[ScenarioState] = []
    scenario_specs: List[ScenarioSpec] = []
    processed_clouds = 0
    skipped_clouds = 0
    max_clouds = int(args.max_clouds)

    for _, cloud in iter_cloud_rows(conn, ids[args.cloud_topic], start_stamp, end_stamp, args.cloud_stride):
        if max_clouds > 0 and processed_clouds >= max_clouds:
            break

        cloud_stamp = stamp_to_sec(cloud.header.stamp)
        truth_pose = interp_truth(cloud_stamp)
        if truth_pose is None:
            skipped_clouds += 1
            continue

        if first_cloud_stamp is None:
            first_cloud_stamp = cloud_stamp
            total_duration = (
                max(0.0, args.duration_sec)
                if args.duration_sec > 0.0
                else max(0.0, truth.poses[-1].stamp - first_cloud_stamp)
            )
            scenario_specs = build_scenario_specs(args, total_duration)
            states = [
                ScenarioState(
                    spec=spec,
                    windows=build_outage_windows(first_cloud_stamp, spec.outage_specs),
                )
                for spec in scenario_specs
            ]

        assert first_cloud_stamp is not None
        elapsed = cloud_stamp - first_cloud_stamp
        scan = None

        for state in states:
            window = outage_at(cloud_stamp, state.windows)
            gnss_available = window is None
            icp = IcpResult(
                pose=truth_pose,
                used=False,
                inliers=0,
                rmse=None,
                median_error=None,
                iterations=0,
                reason="gnss_ins_available",
            )
            scan_points: Optional[int] = None
            keyframes: Optional[int] = None

            if gnss_available:
                state.active_lio = None
                fused_pose = truth_pose
                fused_source = "gnss_ins"
                outage_id = ""
            else:
                assert window is not None
                if state.active_lio is None or state.active_lio.window.outage_id != window.outage_id:
                    anchor = latest_available_gnss_pose_before(truth.poses, window.start_stamp, state.windows)
                    state.active_lio = OutageLioRuntime(window=window, anchor_pose=anchor, keyframes=deque())
                    state.outage_runtime_by_id[window.outage_id] = state.active_lio
                    state.outage_initializations.append(
                        {
                            "outage_id": window.outage_id,
                            "outage_start_t": round(window.start_t, 6),
                            "outage_duration_sec": round(window.duration_sec, 6),
                            "anchor_stamp": round(anchor.stamp, 6),
                            "anchor_t": round(anchor.stamp - first_cloud_stamp, 6),
                            "anchor_x": round(anchor.x, 4),
                            "anchor_y": round(anchor.y, 4),
                            "anchor_yaw": round(anchor.yaw, 6),
                        }
                    )

                if scan is None:
                    scan = extract_scan_2d(cloud, cfg)
                scan_points = int(len(scan))
                state.fallback_scan_points.append(scan_points)
                fused_pose, icp = run_outage_lio_step(state.active_lio, stamp=cloud_stamp, scan=scan, imu=imu, cfg=cfg)
                keyframes = int(len(state.active_lio.keyframes))
                if icp.used:
                    state.fallback_icp_used += 1
                fused_source = "lio_fallback"
                outage_id = window.outage_id

            sample = make_fusion_sample(
                scenario_id=state.spec.scenario_id,
                scenario_label=state.spec.label,
                elapsed=elapsed,
                cloud_stamp=cloud_stamp,
                truth_pose=truth_pose,
                fused_pose=fused_pose,
                fused_source=fused_source,
                gnss_available=gnss_available,
                outage_id=outage_id,
                icp=icp,
                scan_points=scan_points,
                keyframes=keyframes,
                error_sum=state.error_sum,
                sample_count=len(state.samples),
            )
            state.error_sum += float(sample["error_m"])
            state.samples.append(sample)

        processed_clouds += 1

    conn.close()

    if first_cloud_stamp is None or not states:
        raise RuntimeError("No cloud samples were processed; check start/duration/cloud topic settings.")

    runtime_sec = time.time() - started
    scenarios: Dict[str, Dict[str, object]] = {}
    scenario_options: List[Dict[str, object]] = []
    for state in states:
        summary = summarize_scenario(
            state,
            processed_clouds=processed_clouds,
            skipped_clouds=skipped_clouds,
            truth=truth,
            cfg=cfg,
            runtime_sec=runtime_sec,
        )
        scenarios[state.spec.scenario_id] = {
            "scenario_id": state.spec.scenario_id,
            "label": state.spec.label,
            "outage_count": state.spec.outage_count,
            "segment_duration_sec": state.spec.segment_duration_sec,
            "outage_specs": [spec.__dict__ for spec in state.spec.outage_specs],
            "outage_windows": [window.__dict__ for window in state.windows],
            "summary": summary,
            "samples": state.samples,
        }
        scenario_options.append(
            {
                "id": state.spec.scenario_id,
                "label": state.spec.label,
                "outage_count": state.spec.outage_count,
                "segment_duration_sec": state.spec.segment_duration_sec,
                "outage_total_duration_sec": summary["outage_total_duration_sec"],
                "fallback_rmse_m": summary["fallback_rmse_m"],
            }
        )

    default_scenario_id = scenario_options[0]["id"]
    default_scenario = scenarios[str(default_scenario_id)]
    result = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "dataset_dir": str(dataset_dir),
        "db_path": str(db_path),
        "topics": {
            "cloud": args.cloud_topic,
            "imu": args.imu_topic,
            "gnss": args.gnss_topic,
        },
        "bag_topics": {key: metadata[key] for key in sorted(metadata) if key in {args.cloud_topic, args.imu_topic, args.gnss_topic, "/lidar_imu"}},
        "scenario_mode": args.scenario_mode,
        "default_scenario_id": default_scenario_id,
        "scenario_options": scenario_options,
        "scenarios": scenarios,
        "summary": default_scenario["summary"],
        "samples": default_scenario["samples"],
    }

    output_html = Path(args.output_html).expanduser().resolve() if args.output_html else output_dir / "fuse_eval.html"
    write_outputs(result, output_dir, output_html)
    return result


def write_outputs(result: Dict[str, object], output_dir: Path, output_html: Path) -> None:
    summary_path = output_dir / "summary.json"
    trace_path = output_dir / "fused_gnss_lio_trace.csv"
    data_path = output_dir / "fused_gnss_lio_trace.json"
    summary_payload = {
        "generated_at": result.get("generated_at"),
        "dataset_dir": result.get("dataset_dir"),
        "scenario_mode": result.get("scenario_mode"),
        "default_scenario_id": result.get("default_scenario_id"),
        "scenario_options": result.get("scenario_options", []),
        "scenarios": {
            scenario_id: scenario.get("summary", {})
            for scenario_id, scenario in dict(result.get("scenarios", {})).items()
        },
    }
    summary_path.write_text(json.dumps(summary_payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    data_path.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    scenario_items = dict(result.get("scenarios", {})).items()
    if scenario_items:
        fieldnames = [
            "scenario_id",
            "scenario_label",
            "t",
            "stamp",
            "truth_x",
            "truth_y",
            "truth_yaw",
            "fused_x",
            "fused_y",
            "fused_yaw",
            "source",
            "gnss_available",
            "outage_id",
            "error_m",
            "mean_error_m",
            "yaw_error_rad",
            "icp_used",
            "icp_inliers",
            "icp_rmse_m",
            "icp_median_error_m",
            "icp_iterations",
            "icp_reason",
            "scan_points",
            "keyframes",
        ]
        with trace_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for scenario_id, scenario in scenario_items:
                for sample in scenario.get("samples", []):
                    icp = sample["icp"]
                    writer.writerow(
                        {
                            "scenario_id": scenario_id,
                            "scenario_label": scenario.get("label", ""),
                            "t": sample["t"],
                            "stamp": sample["stamp"],
                            "truth_x": sample["truth"]["x"],
                            "truth_y": sample["truth"]["y"],
                            "truth_yaw": sample["truth"]["yaw"],
                            "fused_x": sample["fused"]["x"],
                            "fused_y": sample["fused"]["y"],
                            "fused_yaw": sample["fused"]["yaw"],
                            "source": sample["fused"]["source"],
                            "gnss_available": sample["gnss_available"],
                            "outage_id": sample["outage_id"],
                            "error_m": sample["error_m"],
                            "mean_error_m": sample["mean_error_m"],
                            "yaw_error_rad": sample["yaw_error_rad"],
                            "icp_used": icp["used"],
                            "icp_inliers": icp["inliers"],
                            "icp_rmse_m": icp["rmse_m"],
                            "icp_median_error_m": icp["median_error_m"],
                            "icp_iterations": icp["iterations"],
                            "icp_reason": icp["reason"],
                            "scan_points": "" if sample["scan_points"] is None else sample["scan_points"],
                            "keyframes": "" if sample["keyframes"] is None else sample["keyframes"],
                        }
                    )

    html_payload = dict(result)
    html_payload["output_files"] = {
        "summary_json": str(summary_path),
        "trace_csv": str(trace_path),
        "trace_json": str(data_path),
        "html": str(output_html),
    }
    output_html.parent.mkdir(parents=True, exist_ok=True)
    output_html.write_text(render_html(html_payload), encoding="utf-8")


def render_html(data: Dict[str, object]) -> str:
    payload = html.escape(json.dumps(data, ensure_ascii=False, separators=(",", ":")), quote=False)
    return HTML_TEMPLATE.replace("__FUSE_EVAL_DATA__", payload)


HTML_TEMPLATE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RacingBrain Fusion Evaluation</title>
<style>
html,body{margin:0;height:100%;overflow:hidden;background:#f5f6f1;color:#202427;font:13px/1.35 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
#stage{display:block;width:100vw;height:100vh;background:#f5f6f1;cursor:default}
#controls{position:fixed;left:0;right:0;bottom:0;height:46px;display:flex;align-items:center;gap:12px;padding:0 16px;background:rgba(245,246,241,.96);border-top:1px solid #cfd6ca}
button,select{height:28px;border:1px solid #9ca89c;background:#fff;color:#202427;border-radius:6px;padding:0 11px;font:12px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
#timeline{flex:1;min-width:120px}
#readout{min-width:230px;text-align:right;color:#596058}
</style>
</head>
<body>
<canvas id="stage"></canvas>
<div id="controls">
  <button id="play">Pause</button>
  <input id="timeline" type="range" min="0" max="0" value="0">
  <select id="scenario"></select>
  <select id="speed"><option value="0.25">0.25x</option><option value="0.5">0.5x</option><option value="1" selected>1x</option><option value="2">2x</option><option value="4">4x</option><option value="8">8x</option></select>
  <button id="reset">Reset View</button>
  <span id="readout"></span>
</div>
<script id="fuse-data" type="application/json">__FUSE_EVAL_DATA__</script>
<script>
const data = JSON.parse(document.getElementById("fuse-data").textContent);
const scenarioOptions = data.scenario_options || [{id:data.default_scenario_id || "default", label:"Default"}];
const scenarios = data.scenarios || {};
let activeScenarioId = data.default_scenario_id || scenarioOptions[0].id;
let activeScenario = scenarios[activeScenarioId] || {label:"Default", summary:data.summary || {}, samples:data.samples || []};
let samples = activeScenario.samples || [];
const canvas = document.getElementById("stage");
const ctx = canvas.getContext("2d");
const playBtn = document.getElementById("play");
const timeline = document.getElementById("timeline");
const scenarioSel = document.getElementById("scenario");
const speedSel = document.getElementById("speed");
const resetBtn = document.getElementById("reset");
const readout = document.getElementById("readout");
let dpr = Math.max(1, window.devicePixelRatio || 1);
let playing = true;
let idx = 0;
let lastTs = 0;
let dragging = false;
let dragLast = null;
let view = {cx:0, cy:0, scale:1, fitScale:1};

for(const option of scenarioOptions){
  const el = document.createElement("option");
  el.value = option.id;
  el.textContent = option.label;
  scenarioSel.appendChild(el);
}
scenarioSel.value = activeScenarioId;
timeline.max = String(Math.max(0, samples.length - 1));

function setScenario(id){
  activeScenarioId = id;
  activeScenario = scenarios[id] || activeScenario;
  samples = activeScenario.samples || [];
  idx = 0;
  timeline.max = String(Math.max(0, samples.length - 1));
  timeline.value = "0";
  resetView();
  draw();
}

function resize(){
  dpr = Math.max(1, window.devicePixelRatio || 1);
  canvas.width = Math.floor(innerWidth * dpr);
  canvas.height = Math.floor(innerHeight * dpr);
  canvas.style.width = innerWidth + "px";
  canvas.style.height = innerHeight + "px";
  ctx.setTransform(dpr,0,0,dpr,0,0);
  resetView();
}

function layout(){
  const bottom = 58;
  const pad = 16;
  const side = Math.min(370, Math.max(292, innerWidth * 0.28));
  const mainW = Math.max(340, innerWidth - side - pad * 3);
  const mainH = Math.max(430, innerHeight - bottom - pad * 2);
  const errH = Math.min(210, Math.max(142, mainH * 0.25));
  const gap = 12;
  const mapH = Math.max(260, mainH - errH - gap);
  return {
    map:{x:pad,y:pad,w:mainW,h:mapH},
    err:{x:pad,y:pad+mapH+gap,w:mainW,h:errH},
    panel:{x:pad+mainW+12,y:pad,w:side,h:mainH}
  };
}

function allBounds(){
  let xs=[], ys=[];
  for(const s of samples){
    if(s.truth){xs.push(s.truth.x); ys.push(s.truth.y);}
    if(s.fused){xs.push(s.fused.x); ys.push(s.fused.y);}
  }
  if(!xs.length) return {minX:-10,maxX:10,minY:-10,maxY:10};
  return {minX:Math.min(...xs), maxX:Math.max(...xs), minY:Math.min(...ys), maxY:Math.max(...ys)};
}

function resetView(){
  const r = layout().map;
  const b = allBounds();
  const w = Math.max(1, b.maxX - b.minX);
  const h = Math.max(1, b.maxY - b.minY);
  const scale = Math.min(r.w / (w * 1.18), r.h / (h * 1.18));
  view = {cx:(b.minX+b.maxX)/2, cy:(b.minY+b.maxY)/2, scale:Math.max(0.02, scale), fitScale:Math.max(0.02, scale)};
}

function worldToScreen(p,r){
  return {x:r.x + r.w/2 + (p.x - view.cx) * view.scale, y:r.y + r.h/2 - (p.y - view.cy) * view.scale};
}

function screenToWorld(x,y,r){
  return {x:view.cx + (x - (r.x + r.w/2)) / view.scale, y:view.cy - (y - (r.y + r.h/2)) / view.scale};
}

function panel(rect,title){
  ctx.fillStyle="#ffffff";
  ctx.strokeStyle="#cfd6ca";
  ctx.lineWidth=1;
  ctx.fillRect(rect.x,rect.y,rect.w,rect.h);
  ctx.strokeRect(rect.x+.5,rect.y+.5,rect.w-1,rect.h-1);
  ctx.fillStyle="#202427";
  ctx.font="600 13px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace";
  ctx.fillText(title, rect.x+12, rect.y+22);
}

function clip(rect){
  ctx.beginPath();
  ctx.rect(rect.x,rect.y,rect.w,rect.h);
  ctx.clip();
}

function niceGrid(raw){
  const p = Math.pow(10, Math.floor(Math.log10(Math.max(raw, 1e-6))));
  const n = raw / p;
  if(n < 2) return p;
  if(n < 5) return 2*p;
  return 5*p;
}

function drawGrid(rect){
  const metersPerGrid = niceGrid(80 / Math.max(view.scale, 1e-6));
  const topLeft = screenToWorld(rect.x, rect.y, rect);
  const bottomRight = screenToWorld(rect.x+rect.w, rect.y+rect.h, rect);
  ctx.strokeStyle="#e5e9e1";
  ctx.lineWidth=1;
  ctx.beginPath();
  for(let x=Math.floor(topLeft.x/metersPerGrid)*metersPerGrid; x<=bottomRight.x; x+=metersPerGrid){
    const a=worldToScreen({x:x,y:topLeft.y},rect), b=worldToScreen({x:x,y:bottomRight.y},rect);
    ctx.moveTo(a.x,a.y); ctx.lineTo(b.x,b.y);
  }
  for(let y=Math.floor(bottomRight.y/metersPerGrid)*metersPerGrid; y<=topLeft.y; y+=metersPerGrid){
    const a=worldToScreen({x:topLeft.x,y:y},rect), b=worldToScreen({x:bottomRight.x,y:y},rect);
    ctx.moveTo(a.x,a.y); ctx.lineTo(b.x,b.y);
  }
  ctx.stroke();
}

function drawPlainPath(points, rect, color, upto, width, dash=[]){
  ctx.strokeStyle=color;
  ctx.lineWidth=width;
  ctx.setLineDash(dash);
  ctx.beginPath();
  let started=false;
  const n = Math.min(points.length, upto + 1);
  for(let i=0;i<n;i++){
    const p = points[i];
    if(!p) continue;
    const q = worldToScreen(p, rect);
    if(!started){ctx.moveTo(q.x,q.y); started=true;} else {ctx.lineTo(q.x,q.y);}
  }
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawSegmentedPath(rect, pointFn, colorFn, upto, width){
  let prevColor = null;
  let prevPoint = null;
  let started = false;
  ctx.lineWidth = width;
  const n = Math.min(samples.length, upto + 1);
  for(let i=0;i<n;i++){
    const p = pointFn(samples[i]);
    if(!p) continue;
    const color = colorFn(samples[i]);
    const q = worldToScreen(p, rect);
    if(color !== prevColor){
      if(started) ctx.stroke();
      ctx.beginPath();
      ctx.strokeStyle = color;
      if(prevPoint){
        ctx.moveTo(prevPoint.x,prevPoint.y);
        ctx.lineTo(q.x,q.y);
      } else {
        ctx.moveTo(q.x,q.y);
      }
      started = true;
      prevColor = color;
    } else {
      ctx.lineTo(q.x,q.y);
    }
    prevPoint = q;
  }
  if(started) ctx.stroke();
}

function legend(x,y,items){
  ctx.font="12px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace";
  let offset=0;
  for(const [color,label,dashed] of items){
    ctx.strokeStyle=color;
    ctx.lineWidth=3;
    ctx.setLineDash(dashed ? [8,5] : []);
    ctx.beginPath(); ctx.moveTo(x+offset,y-7); ctx.lineTo(x+offset+22,y-7); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle="#596058"; ctx.fillText(label,x+offset+24,y-4);
    offset += ctx.measureText(label).width + 58;
  }
}

function drawMapShell(rect,title){
  panel(rect,title);
  ctx.save();
  clip({x:rect.x+1,y:rect.y+30,w:rect.w-2,h:rect.h-31});
  drawGrid(rect);
  ctx.restore();
}

function drawFusionMap(rect){
  drawMapShell(rect, "Complete GNSS/INS truth vs fused trajectory");
  const truth = samples.map(s => s.truth);
  ctx.save();
  clip({x:rect.x+1,y:rect.y+30,w:rect.w-2,h:rect.h-31});
  drawPlainPath(truth, rect, "#148a5a", samples.length - 1, 4.0, [14,8]);
  drawSegmentedPath(rect, s => s.fused, s => s.gnss_available ? "#2f79bd" : "#d97a2b", samples.length - 1, 3.0);
  const cur = samples[idx];
  if(cur && cur.truth && cur.fused){
    const truthPoint = worldToScreen(cur.truth, rect);
    const fusedPoint = worldToScreen(cur.fused, rect);
    ctx.strokeStyle="#cc3b2e";
    ctx.lineWidth=1.4;
    ctx.beginPath(); ctx.moveTo(truthPoint.x,truthPoint.y); ctx.lineTo(fusedPoint.x,fusedPoint.y); ctx.stroke();
    ctx.fillStyle="#202427";
    ctx.beginPath(); ctx.arc(truthPoint.x,truthPoint.y,5,0,Math.PI*2); ctx.fill();
    ctx.strokeStyle="#fff"; ctx.lineWidth=2; ctx.stroke();
    ctx.fillStyle=cur.gnss_available ? "#2f79bd" : "#d97a2b";
    ctx.beginPath(); ctx.arc(fusedPoint.x,fusedPoint.y,5,0,Math.PI*2); ctx.fill();
    ctx.strokeStyle="#fff"; ctx.lineWidth=2; ctx.stroke();
  }
  ctx.restore();
  legend(rect.x+12, rect.y+42, [["#148a5a","GNSS/INS truth",true],["#2f79bd","fused GNSS/INS",false],["#d97a2b","fused LIO fallback",false],["#cc3b2e","current error",false]]);
}

function drawErrors(rect){
  panel(rect, "Position error");
  legend(rect.x+150, rect.y+24, [["#cc3b2e","instant",false],["#2f79bd","cumulative mean",false],["#d97a2b","outage",false]]);
  const plot = {x:rect.x+52,y:rect.y+48,w:rect.w-78,h:rect.h-70};
  ctx.strokeStyle="#d6ddd2"; ctx.strokeRect(plot.x+.5,plot.y+.5,plot.w-1,plot.h-1);
  const maxErr = Math.max(1, ...samples.map(s => Math.max(s.error_m || 0, s.mean_error_m || 0)));
  function px(i){ return plot.x + (samples.length <= 1 ? 0 : i/(samples.length-1))*plot.w; }
  function py(v){ return plot.y + plot.h - (v/maxErr)*plot.h; }
  for(let i=0;i<samples.length;i++){
    if(!samples[i].gnss_available){
      let j=i;
      while(j+1<samples.length && !samples[j+1].gnss_available) j++;
      ctx.fillStyle="rgba(217,122,43,.16)";
      ctx.fillRect(px(i), plot.y, Math.max(1, px(j)-px(i)), plot.h);
      i=j;
    }
  }
  function curve(key,color,width){
    ctx.strokeStyle=color; ctx.lineWidth=width; ctx.beginPath();
    for(let i=0;i<samples.length;i++){
      const x=px(i), y=py(samples[i][key] || 0);
      if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }
  curve("error_m","#cc3b2e",1.8);
  curve("mean_error_m","#2f79bd",1.8);
  const x=px(idx);
  ctx.strokeStyle="#202427"; ctx.lineWidth=1; ctx.beginPath(); ctx.moveTo(x,plot.y); ctx.lineTo(x,plot.y+plot.h); ctx.stroke();
  ctx.fillStyle="#596058"; ctx.font="11px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace";
  ctx.fillText(maxErr.toFixed(1)+" m", rect.x+8, plot.y+4);
  ctx.fillText("0", rect.x+34, plot.y+plot.h-2);
}

function fmt(v,d=3){
  if(v === null || v === undefined || !Number.isFinite(Number(v))) return "n/a";
  return Number(v).toFixed(d);
}

function drawPanel(rect){
  panel(rect, "Values");
  const s = samples[idx] || {};
  const sum = activeScenario.summary || data.summary || {};
  const pos = (sum.position_error_m || {});
  const fallback = (sum.fallback_position_error_m || {});
  const rows = [
    ["Severity", activeScenario.label || activeScenarioId],
    ["Samples", String(samples.length)],
    ["Elapsed", fmt(s.t,2)+" s"],
    ["Source", (s.fused && s.fused.source) || "n/a"],
    ["GNSS avail", String(s.gnss_available === true)],
    ["Outage", s.outage_id || "none"],
    ["Truth x/y", fmt(s.truth && s.truth.x,2)+", "+fmt(s.truth && s.truth.y,2)],
    ["Fused x/y", fmt(s.fused && s.fused.x,2)+", "+fmt(s.fused && s.fused.y,2)],
    ["Error", fmt(s.error_m,3)+" m"],
    ["Mean error", fmt(s.mean_error_m,3)+" m"],
    ["Max error", fmt(pos.max,3)+" m"],
    ["RMSE", fmt(pos.rmse,3)+" m"],
    ["Fallback n", String(sum.fallback_sample_count ?? "n/a")],
    ["Fallback RMSE", fmt(fallback.rmse,3)+" m"],
    ["Gap length", fmt(sum.segment_duration_sec,1)+" s"],
    ["Outage total", fmt(sum.outage_total_duration_sec,2)+" s"],
    ["Outages", String(sum.outage_count ?? "n/a")]
  ];
  ctx.font="12px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace";
  let y = rect.y + 50;
  for(const [k,v] of rows){
    ctx.fillStyle="#6a7168"; ctx.fillText(k, rect.x+14, y);
    ctx.fillStyle="#202427"; ctx.fillText(String(v), rect.x+136, y);
    y += 24;
  }
  ctx.fillStyle="#6a7168";
  const note = (sum.method && sum.method.fallback_rule) || "";
  wrapText(note, rect.x+14, rect.y+rect.h-82, rect.w-28, 16);
}

function wrapText(text,x,y,w,lineH){
  const words = String(text).split(/\\s+/);
  let line="";
  for(const word of words){
    const test = line ? line + " " + word : word;
    if(ctx.measureText(test).width > w && line){
      ctx.fillText(line,x,y); y += lineH; line = word;
    } else {
      line = test;
    }
  }
  if(line) ctx.fillText(line,x,y);
}

function draw(){
  ctx.clearRect(0,0,innerWidth,innerHeight);
  const r = layout();
  drawFusionMap(r.map);
  drawErrors(r.err);
  drawPanel(r.panel);
  const cur = samples[idx];
  readout.textContent = cur ? `${activeScenario.label || activeScenarioId}  ${idx+1}/${samples.length}  ${fmt(cur.t,2)}s  ${cur.fused.source}  ${fmt(cur.error_m,2)}m` : "0/0";
  timeline.value = String(idx);
}

function tick(ts){
  if(!lastTs) lastTs = ts;
  const dt = (ts-lastTs)/1000;
  lastTs = ts;
  if(playing && samples.length > 1){
    const speed = Number(speedSel.value || 1);
    const duration = Math.max(0.001, samples[samples.length-1].t - samples[0].t);
    const step = dt * speed / duration * (samples.length - 1);
    idx = Math.min(samples.length-1, idx + step);
    if(idx >= samples.length-1){ idx = samples.length-1; playing=false; playBtn.textContent="Play"; }
  }
  idx = Math.round(idx);
  draw();
  requestAnimationFrame(tick);
}

function hitMap(x,y){
  const r = layout();
  const rect = r.map;
  if(x >= rect.x && x <= rect.x+rect.w && y >= rect.y && y <= rect.y+rect.h) return rect;
  return null;
}

playBtn.onclick = () => {
  playing = !playing;
  if(playing && idx >= samples.length-1) idx = 0;
  playBtn.textContent = playing ? "Pause" : "Play";
};
scenarioSel.onchange = () => { playing = false; playBtn.textContent = "Play"; setScenario(scenarioSel.value); };
timeline.oninput = () => { idx = Number(timeline.value); playing = false; playBtn.textContent = "Play"; draw(); };
resetBtn.onclick = () => { resetView(); draw(); };

canvas.addEventListener("mousedown", ev => {
  if(hitMap(ev.clientX, ev.clientY)){
    dragging = true; dragLast = {x:ev.clientX,y:ev.clientY};
  }
});
addEventListener("mouseup", () => { dragging = false; dragLast = null; });
addEventListener("mousemove", ev => {
  if(!dragging || !dragLast) return;
  view.cx -= (ev.clientX - dragLast.x) / view.scale;
  view.cy += (ev.clientY - dragLast.y) / view.scale;
  dragLast = {x:ev.clientX,y:ev.clientY};
  draw();
});
canvas.addEventListener("wheel", ev => {
  const rect = hitMap(ev.clientX, ev.clientY);
  if(!rect) return;
  ev.preventDefault();
  const before = screenToWorld(ev.clientX, ev.clientY, rect);
  const factor = ev.deltaY < 0 ? 1.15 : 1/1.15;
  const minScale = Math.max(0.001, view.fitScale * 0.04);
  const maxScale = Math.max(400, view.fitScale * 80);
  view.scale = Math.min(maxScale, Math.max(minScale, view.scale * factor));
  const after = screenToWorld(ev.clientX, ev.clientY, rect);
  view.cx += before.x - after.x;
  view.cy += before.y - after.y;
  draw();
},{passive:false});

window.addEventListener("resize", resize);
resize();
requestAnimationFrame(tick);
</script>
</body>
</html>
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate GNSS/INS-priority localization with simple LIO fallback during artificial outages.")
    parser.add_argument("--dataset-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--output-html", default="")
    parser.add_argument("--cloud-topic", default="/lidar_points")
    parser.add_argument("--imu-topic", default="/imu")
    parser.add_argument("--gnss-topic", default="/gongji_gnss_ins_64")
    parser.add_argument("--scenario-mode", choices=("severity_grid", "single"), default="severity_grid")
    parser.add_argument("--outages", default=DEFAULT_OUTAGES, help="Comma-separated start:duration specs in seconds, relative to first processed cloud. Example: 45:8,185:10,gap_turn@360:8")
    parser.add_argument("--gap-count", type=int, default=None, help="Run one generated scenario with this many GNSS/INS gaps.")
    parser.add_argument("--gap-duration-sec", type=float, default=None, help="Run one generated scenario with this gap duration in seconds.")
    parser.add_argument("--start-sec", type=float, default=0.0)
    parser.add_argument("--duration-sec", type=float, default=0.0, help="0 means full available duration.")
    parser.add_argument("--max-clouds", type=int, default=0, help="0 means no cloud-count limit.")
    parser.add_argument("--cloud-stride", type=int, default=1)
    parser.add_argument("--lidar-min-range", type=float, default=2.0)
    parser.add_argument("--lidar-max-range", type=float, default=90.0)
    parser.add_argument("--z-min", type=float, default=-0.5)
    parser.add_argument("--z-max", type=float, default=4.0)
    parser.add_argument("--icp-z-weight", type=float, default=0.7)
    parser.add_argument("--scan-voxel", type=float, default=0.35)
    parser.add_argument("--map-voxel", type=float, default=0.35)
    parser.add_argument("--max-scan-points", type=int, default=6500)
    parser.add_argument("--max-keyframe-points", type=int, default=4500)
    parser.add_argument("--local-map-keyframes", type=int, default=35)
    parser.add_argument("--keyframe-dist", type=float, default=0.45)
    parser.add_argument("--keyframe-yaw", type=float, default=0.08)
    parser.add_argument("--keyframe-time", type=float, default=1.2)
    parser.add_argument("--icp-iterations", type=int, default=10)
    parser.add_argument("--icp-max-correspondence", type=float, default=2.0)
    parser.add_argument("--icp-trim-fraction", type=float, default=0.82)
    parser.add_argument("--icp-min-inliers", type=int, default=80)
    parser.add_argument("--icp-max-rmse", type=float, default=1.5)
    parser.add_argument("--icp-max-translation-step", type=float, default=0.45)
    parser.add_argument("--icp-max-yaw-step", type=float, default=0.08)
    parser.add_argument("--icp-max-pose-correction", type=float, default=0.55)
    parser.add_argument("--icp-max-pose-correction-yaw", type=float, default=0.06)
    parser.add_argument("--imu-gyro-scale", type=float, default=math.pi / 180.0, help="Scale raw IMU angular_velocity.z to rad/s.")
    parser.add_argument("--max-frame-dt", type=float, default=0.35)
    parser.add_argument("--max-speed", type=float, default=4.0)
    parser.add_argument("--max-yaw-rate", type=float, default=1.0)
    parser.add_argument("--sparse-scan-points", type=int, default=1200)
    parser.add_argument("--sparse-icp-weight", type=float, default=0.25)
    parser.add_argument("--velocity-update-alpha", type=float, default=0.45)
    parser.add_argument("--sparse-velocity-update-alpha", type=float, default=0.05)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = evaluate_dataset(args)
    print(
        json.dumps(
            {
                "default_scenario_id": result["default_scenario_id"],
                "scenario_options": result["scenario_options"],
                "scenarios": {
                    scenario_id: scenario["summary"]
                    for scenario_id, scenario in result["scenarios"].items()
                },
            },
            indent=2,
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
