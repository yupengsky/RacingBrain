#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import html
import json
import math
import sqlite3
import statistics
import time
from collections import Counter, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
from gnss_ins_msg.msg import Gnssins64
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2

from racingbrain.localization.local_tangent import LocalTangentProjector
from racingbrain.localization.pose_sources import wrap_angle


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def finite_stats(values: Sequence[float]) -> Dict[str, Optional[float]]:
    clean = [float(value) for value in values if math.isfinite(float(value))]
    if not clean:
        return {"mean": None, "median": None, "p95": None, "max": None, "rmse": None}
    ordered = sorted(clean)
    p95_index = min(len(ordered) - 1, max(0, math.ceil(0.95 * len(ordered)) - 1))
    return {
        "mean": round(statistics.fmean(clean), 6),
        "median": round(statistics.median(clean), 6),
        "p95": round(ordered[p95_index], 6),
        "max": round(max(clean), 6),
        "rmse": round(math.sqrt(statistics.fmean([value * value for value in clean])), 6),
    }


def round_or_none(value: Optional[float], digits: int = 6) -> Optional[float]:
    if value is None or not math.isfinite(float(value)):
        return None
    return round(float(value), digits)


@dataclass
class Pose2:
    stamp: float
    x: float
    y: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0

    def copy_at(self, stamp: float) -> "Pose2":
        return Pose2(stamp=stamp, x=self.x, y=self.y, yaw=self.yaw, vx=self.vx, vy=self.vy)


@dataclass
class GnssTruth:
    poses: List[Pose2]
    accuracy_horizon_m: List[float]
    accuracy_yaw_deg: List[float]
    ins_status: List[int]
    gnss_status: List[int]
    satellite_main: List[int]
    satellite_sub: List[int]


@dataclass
class ImuSeries:
    stamps: np.ndarray
    yaw_rates: np.ndarray
    yaw_integral: np.ndarray

    def yaw_delta(self, start: float, end: float) -> float:
        if len(self.stamps) < 2 or end <= start:
            return 0.0
        lo = float(np.clip(start, self.stamps[0], self.stamps[-1]))
        hi = float(np.clip(end, self.stamps[0], self.stamps[-1]))
        if hi <= lo:
            return 0.0
        return float(np.interp(hi, self.stamps, self.yaw_integral) - np.interp(lo, self.stamps, self.yaw_integral))


@dataclass
class IcpResult:
    pose: Pose2
    used: bool
    inliers: int
    rmse: Optional[float]
    median_error: Optional[float]
    iterations: int
    reason: str


@dataclass
class Keyframe:
    stamp: float
    x: float
    y: float
    yaw: float
    points_world: np.ndarray


@dataclass
class SimpleLioConfig:
    lidar_min_range_m: float
    lidar_max_range_m: float
    z_min_m: float
    z_max_m: float
    icp_z_weight: float
    scan_voxel_m: float
    map_voxel_m: float
    max_scan_points: int
    max_keyframe_points: int
    local_map_keyframes: int
    keyframe_dist_m: float
    keyframe_yaw_rad: float
    keyframe_time_sec: float
    icp_iterations: int
    icp_max_correspondence_m: float
    icp_trim_fraction: float
    icp_min_inliers: int
    icp_max_rmse_m: float
    icp_max_translation_step_m: float
    icp_max_yaw_step_rad: float
    icp_max_pose_correction_m: float
    icp_max_pose_correction_yaw_rad: float
    imu_gyro_scale: float
    max_frame_dt_sec: float
    max_speed_mps: float
    max_yaw_rate_radps: float
    sparse_scan_points: int
    sparse_icp_weight: float
    velocity_update_alpha: float
    sparse_velocity_update_alpha: float


def open_bag_db(dataset_dir: Path) -> Path:
    db_paths = sorted(dataset_dir.glob("*.db3"))
    if not db_paths:
        raise FileNotFoundError(f"No .db3 rosbag storage file found under {dataset_dir}")
    if len(db_paths) > 1:
        raise RuntimeError(f"This simple evaluator expects one db3 file; found {len(db_paths)} in {dataset_dir}")
    return db_paths[0]


def topic_ids(conn: sqlite3.Connection) -> Dict[str, int]:
    return {name: int(topic_id) for topic_id, name in conn.execute("select id,name from topics")}


def topic_metadata(conn: sqlite3.Connection) -> Dict[str, Dict[str, object]]:
    rows = conn.execute(
        """
        select topics.name, topics.type, count(messages.id), min(messages.timestamp), max(messages.timestamp)
        from topics left join messages on messages.topic_id = topics.id
        group by topics.id
        order by topics.name
        """
    )
    out: Dict[str, Dict[str, object]] = {}
    for name, msg_type, count, first_ns, last_ns in rows:
        duration = None if first_ns is None or last_ns is None else (int(last_ns) - int(first_ns)) * 1e-9
        rate = None
        if duration and duration > 0.0 and count and count > 1:
            rate = (int(count) - 1) / duration
        out[str(name)] = {
            "type": str(msg_type),
            "count": int(count or 0),
            "first_stamp_sec": None if first_ns is None else round(int(first_ns) * 1e-9, 6),
            "last_stamp_sec": None if last_ns is None else round(int(last_ns) * 1e-9, 6),
            "duration_sec": round(duration, 6) if duration is not None else None,
            "rate_hz": round(rate, 6) if rate is not None else None,
        }
    return out


def read_gnss_truth(conn: sqlite3.Connection, topic_id: int) -> GnssTruth:
    projector = LocalTangentProjector()
    poses: List[Pose2] = []
    accuracy_horizon: List[float] = []
    accuracy_yaw: List[float] = []
    ins_status: List[int] = []
    gnss_status: List[int] = []
    satellite_main: List[int] = []
    satellite_sub: List[int] = []

    for (data,) in conn.execute("select data from messages where topic_id=? order by timestamp", (topic_id,)):
        msg = deserialize_message(data, Gnssins64)
        stamp = stamp_to_sec(msg.header.stamp)
        if stamp <= 0.0:
            continue
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        if not (math.isfinite(lat) and math.isfinite(lon)) or (abs(lat) < 1e-12 and abs(lon) < 1e-12):
            continue
        x, y = projector.forward(lat, lon)
        yaw = wrap_angle(math.radians(float(msg.yaw)) + math.pi * 0.5)
        poses.append(Pose2(stamp=stamp, x=x, y=y, yaw=yaw, vx=float(msg.vel_e), vy=float(msg.vel_n)))
        accuracy_horizon.append(float(msg.accuracy_horizon))
        accuracy_yaw.append(float(msg.accuracy_yaw))
        ins_status.append(int(msg.ins_status))
        gnss_status.append(int(msg.gnss_status))
        satellite_main.append(int(msg.satellite_main))
        satellite_sub.append(int(msg.satellite_sub))

    if len(poses) < 2:
        raise RuntimeError("GNSS/INS truth topic has fewer than two valid samples")
    return GnssTruth(
        poses=poses,
        accuracy_horizon_m=accuracy_horizon,
        accuracy_yaw_deg=accuracy_yaw,
        ins_status=ins_status,
        gnss_status=gnss_status,
        satellite_main=satellite_main,
        satellite_sub=satellite_sub,
    )


def read_imu_series(conn: sqlite3.Connection, topic_id: int, gyro_scale: float) -> ImuSeries:
    stamps: List[float] = []
    yaw_rates: List[float] = []
    for (data,) in conn.execute("select data from messages where topic_id=? order by timestamp", (topic_id,)):
        msg = deserialize_message(data, Imu)
        stamp = stamp_to_sec(msg.header.stamp)
        if stamp <= 0.0:
            continue
        yaw_rate = float(msg.angular_velocity.z) * gyro_scale
        if math.isfinite(yaw_rate):
            stamps.append(stamp)
            yaw_rates.append(yaw_rate)

    if len(stamps) < 2:
        return ImuSeries(np.asarray([], dtype=np.float64), np.asarray([], dtype=np.float64), np.asarray([], dtype=np.float64))

    ts = np.asarray(stamps, dtype=np.float64)
    rates = np.asarray(yaw_rates, dtype=np.float64)
    order = np.argsort(ts)
    ts = ts[order]
    rates = rates[order]
    dt = np.diff(ts)
    increments = 0.5 * (rates[:-1] + rates[1:]) * np.clip(dt, 0.0, 0.2)
    integral = np.concatenate(([0.0], np.cumsum(increments)))
    return ImuSeries(ts, rates, integral)


def make_gnss_interpolator(truth: GnssTruth):
    stamps = np.asarray([pose.stamp for pose in truth.poses], dtype=np.float64)
    xs = np.asarray([pose.x for pose in truth.poses], dtype=np.float64)
    ys = np.asarray([pose.y for pose in truth.poses], dtype=np.float64)
    yaws = np.asarray([pose.yaw for pose in truth.poses], dtype=np.float64)
    vxs = np.asarray([pose.vx for pose in truth.poses], dtype=np.float64)
    vys = np.asarray([pose.vy for pose in truth.poses], dtype=np.float64)

    def interp(stamp: float) -> Optional[Pose2]:
        if stamp < stamps[0] or stamp > stamps[-1]:
            return None
        idx = int(np.searchsorted(stamps, stamp))
        if idx <= 0:
            return Pose2(stamp=stamp, x=float(xs[0]), y=float(ys[0]), yaw=float(yaws[0]), vx=float(vxs[0]), vy=float(vys[0]))
        if idx >= len(stamps):
            return Pose2(stamp=stamp, x=float(xs[-1]), y=float(ys[-1]), yaw=float(yaws[-1]), vx=float(vxs[-1]), vy=float(vys[-1]))
        t0 = stamps[idx - 1]
        t1 = stamps[idx]
        alpha = float((stamp - t0) / max(t1 - t0, 1e-9))
        yaw = wrap_angle(float(yaws[idx - 1]) + wrap_angle(float(yaws[idx] - yaws[idx - 1])) * alpha)
        return Pose2(
            stamp=stamp,
            x=float(xs[idx - 1] + (xs[idx] - xs[idx - 1]) * alpha),
            y=float(ys[idx - 1] + (ys[idx] - ys[idx - 1]) * alpha),
            yaw=yaw,
            vx=float(vxs[idx - 1] + (vxs[idx] - vxs[idx - 1]) * alpha),
            vy=float(vys[idx - 1] + (vys[idx] - vys[idx - 1]) * alpha),
        )

    return interp


def voxel_downsample(points: np.ndarray, voxel_size: float, max_points: int = 0) -> np.ndarray:
    if points.size == 0:
        width = points.shape[1] if points.ndim == 2 else 3
        return points.reshape(0, width)
    if voxel_size > 0.0:
        keys = np.floor(points / voxel_size).astype(np.int64)
        _, index = np.unique(keys, axis=0, return_index=True)
        points = points[np.sort(index)]
    if max_points > 0 and len(points) > max_points:
        index = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
        points = points[index]
    return np.ascontiguousarray(points, dtype=np.float64)


def extract_scan_2d(msg: PointCloud2, cfg: SimpleLioConfig) -> np.ndarray:
    dtype = point_cloud2.dtype_from_fields(msg.fields, point_step=msg.point_step)
    raw = np.frombuffer(msg.data, dtype=dtype, count=int(msg.width) * int(msg.height))
    if raw.size == 0:
        return np.empty((0, 3), dtype=np.float64)

    x = raw["x"].astype(np.float32, copy=False)
    y = raw["y"].astype(np.float32, copy=False)
    z = raw["z"].astype(np.float32, copy=False)
    horizontal_range = np.hypot(x, y)
    mask = (
        np.isfinite(x)
        & np.isfinite(y)
        & np.isfinite(z)
        & (horizontal_range >= cfg.lidar_min_range_m)
        & (horizontal_range <= cfg.lidar_max_range_m)
        & (z >= cfg.z_min_m)
        & (z <= cfg.z_max_m)
    )
    if not np.any(mask):
        return np.empty((0, 3), dtype=np.float64)
    points = np.column_stack((x[mask], y[mask], z[mask] * cfg.icp_z_weight)).astype(np.float64, copy=False)
    return voxel_downsample(points, cfg.scan_voxel_m, cfg.max_scan_points)


def transform_points(points: np.ndarray, pose: Pose2) -> np.ndarray:
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    out = np.empty_like(points, dtype=np.float64)
    out[:, 0] = c * points[:, 0] - s * points[:, 1] + pose.x
    out[:, 1] = s * points[:, 0] + c * points[:, 1] + pose.y
    if points.shape[1] > 2:
        out[:, 2:] = points[:, 2:]
    return out


def solve_se2(source_points: np.ndarray, target_points: np.ndarray, stamp: float) -> Optional[Pose2]:
    if len(source_points) < 3 or len(target_points) < 3:
        return None
    source_xy = source_points[:, :2]
    target_xy = target_points[:, :2]
    source_centroid = np.mean(source_xy, axis=0)
    target_centroid = np.mean(target_xy, axis=0)
    source_centered = source_xy - source_centroid
    target_centered = target_xy - target_centroid
    h = source_centered.T @ target_centered
    try:
        u, _, vt = np.linalg.svd(h)
    except np.linalg.LinAlgError:
        return None
    r = vt.T @ u.T
    if np.linalg.det(r) < 0.0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T
    yaw = math.atan2(float(r[1, 0]), float(r[0, 0]))
    translation = target_centroid - r @ source_centroid
    return Pose2(stamp=stamp, x=float(translation[0]), y=float(translation[1]), yaw=wrap_angle(yaw))


def clamp_pose_update(current: Pose2, target: Pose2, cfg: SimpleLioConfig) -> Pose2:
    dx = target.x - current.x
    dy = target.y - current.y
    distance = math.hypot(dx, dy)
    if distance > cfg.icp_max_translation_step_m > 0.0:
        scale = cfg.icp_max_translation_step_m / max(distance, 1e-9)
        dx *= scale
        dy *= scale
    dyaw = wrap_angle(target.yaw - current.yaw)
    if abs(dyaw) > cfg.icp_max_yaw_step_rad > 0.0:
        dyaw = math.copysign(cfg.icp_max_yaw_step_rad, dyaw)
    return Pose2(stamp=current.stamp, x=current.x + dx, y=current.y + dy, yaw=wrap_angle(current.yaw + dyaw), vx=current.vx, vy=current.vy)


def run_icp(scan_points: np.ndarray, map_points: np.ndarray, initial_pose: Pose2, cfg: SimpleLioConfig) -> IcpResult:
    if len(scan_points) < cfg.icp_min_inliers:
        return IcpResult(initial_pose, False, 0, None, None, 0, "scan_too_sparse")
    if len(map_points) < cfg.icp_min_inliers:
        return IcpResult(initial_pose, False, 0, None, None, 0, "map_too_sparse")

    tree = cKDTree(map_points)
    pose = initial_pose.copy_at(initial_pose.stamp)
    last_inliers = 0
    last_rmse: Optional[float] = None
    last_median: Optional[float] = None
    reason = "not_converged"
    iterations = 0

    for iteration in range(cfg.icp_iterations):
        world = transform_points(scan_points, pose)
        distances, indices = tree.query(world, k=1, distance_upper_bound=cfg.icp_max_correspondence_m, workers=-1)
        valid = np.isfinite(distances) & (indices < len(map_points))
        valid_distances = distances[valid]
        if len(valid_distances) < cfg.icp_min_inliers:
            reason = "too_few_correspondences"
            break
        if 0.0 < cfg.icp_trim_fraction < 1.0:
            cutoff = min(cfg.icp_max_correspondence_m, float(np.quantile(valid_distances, cfg.icp_trim_fraction)))
            valid = valid & (distances <= cutoff)
            valid_distances = distances[valid]
        if len(valid_distances) < cfg.icp_min_inliers:
            reason = "too_few_trimmed_correspondences"
            break

        source = scan_points[valid]
        target = map_points[indices[valid]]
        candidate = solve_se2(source, target, pose.stamp)
        if candidate is None:
            reason = "svd_failed"
            break

        next_pose = clamp_pose_update(pose, candidate, cfg)
        move = math.hypot(next_pose.x - pose.x, next_pose.y - pose.y)
        turn = abs(wrap_angle(next_pose.yaw - pose.yaw))
        pose = next_pose
        iterations = iteration + 1
        last_inliers = int(len(valid_distances))
        last_rmse = float(math.sqrt(np.mean(valid_distances * valid_distances)))
        last_median = float(np.median(valid_distances))
        if move < 0.003 and turn < 0.0005:
            reason = "converged"
            break

    used = last_inliers >= cfg.icp_min_inliers and last_rmse is not None and last_rmse <= cfg.icp_max_rmse_m
    if not used and last_rmse is not None and last_rmse > cfg.icp_max_rmse_m:
        reason = "rmse_rejected"
    if used:
        correction = math.hypot(pose.x - initial_pose.x, pose.y - initial_pose.y)
        yaw_correction = abs(wrap_angle(pose.yaw - initial_pose.yaw))
        clamped = False
        if correction > cfg.icp_max_pose_correction_m > 0.0:
            scale = cfg.icp_max_pose_correction_m / max(correction, 1e-9)
            pose = Pose2(
                stamp=pose.stamp,
                x=initial_pose.x + (pose.x - initial_pose.x) * scale,
                y=initial_pose.y + (pose.y - initial_pose.y) * scale,
                yaw=pose.yaw,
                vx=pose.vx,
                vy=pose.vy,
            )
            clamped = True
        if yaw_correction > cfg.icp_max_pose_correction_yaw_rad > 0.0:
            yaw_delta = wrap_angle(pose.yaw - initial_pose.yaw)
            pose.yaw = wrap_angle(
                initial_pose.yaw
                + math.copysign(min(abs(yaw_delta), cfg.icp_max_pose_correction_yaw_rad), yaw_delta)
            )
            clamped = True
        if clamped:
            reason = f"{reason}_pose_correction_clamped"
    return IcpResult(pose if used else initial_pose, used, last_inliers, last_rmse, last_median, iterations, reason)


def build_local_map(keyframes: Deque[Keyframe], current_pose: Pose2, cfg: SimpleLioConfig) -> np.ndarray:
    while len(keyframes) > cfg.local_map_keyframes:
        keyframes.popleft()
    if not keyframes:
        return np.empty((0, 3), dtype=np.float64)
    clouds = [frame.points_world for frame in keyframes if len(frame.points_world)]
    if not clouds:
        return np.empty((0, 3), dtype=np.float64)
    points = np.vstack(clouds)
    return voxel_downsample(points, cfg.map_voxel_m, 0)


def should_add_keyframe(keyframes: Deque[Keyframe], pose: Pose2, cfg: SimpleLioConfig) -> bool:
    if not keyframes:
        return True
    last = keyframes[-1]
    dist = math.hypot(pose.x - last.x, pose.y - last.y)
    yaw = abs(wrap_angle(pose.yaw - last.yaw))
    dt = pose.stamp - last.stamp
    return dist >= cfg.keyframe_dist_m or yaw >= cfg.keyframe_yaw_rad or dt >= cfg.keyframe_time_sec


def predict_pose(previous: Pose2, previous_previous: Optional[Pose2], stamp: float, imu: ImuSeries, cfg: SimpleLioConfig) -> Pose2:
    dt = max(0.0, stamp - previous.stamp)
    if dt > cfg.max_frame_dt_sec:
        dt = 0.0
    vx = previous.vx
    vy = previous.vy
    if previous_previous is not None and math.hypot(vx, vy) < 1e-6:
        prev_dt = previous.stamp - previous_previous.stamp
        if 1e-3 < prev_dt <= cfg.max_frame_dt_sec:
            vx = (previous.x - previous_previous.x) / prev_dt
            vy = (previous.y - previous_previous.y) / prev_dt
    speed = math.hypot(vx, vy)
    if speed > cfg.max_speed_mps > 0.0:
        scale = cfg.max_speed_mps / max(speed, 1e-9)
        vx *= scale
        vy *= scale
    yaw = wrap_angle(previous.yaw + imu.yaw_delta(previous.stamp, stamp))
    return Pose2(stamp=stamp, x=previous.x + vx * dt, y=previous.y + vy * dt, yaw=yaw, vx=vx, vy=vy)


def clamp_motion(previous: Pose2, pose: Pose2, cfg: SimpleLioConfig) -> Tuple[Pose2, bool]:
    dt = pose.stamp - previous.stamp
    if dt <= 1e-3 or dt > cfg.max_frame_dt_sec:
        return pose, False
    clamped = False
    max_step = cfg.max_speed_mps * dt if cfg.max_speed_mps > 0.0 else 0.0
    dx = pose.x - previous.x
    dy = pose.y - previous.y
    step = math.hypot(dx, dy)
    if max_step > 0.0 and step > max_step:
        scale = max_step / max(step, 1e-9)
        pose = Pose2(
            stamp=pose.stamp,
            x=previous.x + dx * scale,
            y=previous.y + dy * scale,
            yaw=pose.yaw,
            vx=pose.vx,
            vy=pose.vy,
        )
        clamped = True
    max_yaw_step = cfg.max_yaw_rate_radps * dt if cfg.max_yaw_rate_radps > 0.0 else 0.0
    dyaw = wrap_angle(pose.yaw - previous.yaw)
    if max_yaw_step > 0.0 and abs(dyaw) > max_yaw_step:
        pose.yaw = wrap_angle(previous.yaw + math.copysign(max_yaw_step, dyaw))
        clamped = True
    return pose, clamped


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


def iter_cloud_rows(conn: sqlite3.Connection, topic_id: int, start_stamp: float, end_stamp: Optional[float], stride: int):
    stride = max(1, int(stride))
    index = 0
    query = "select timestamp,data from messages where topic_id=? order by timestamp"
    for bag_ns, data in conn.execute(query, (topic_id,)):
        if index % stride != 0:
            index += 1
            continue
        index += 1
        msg = deserialize_message(data, PointCloud2)
        stamp = stamp_to_sec(msg.header.stamp)
        if stamp < start_stamp:
            continue
        if end_stamp is not None and stamp > end_stamp:
            break
        yield int(bag_ns), msg


def status_counts(values: Iterable[int]) -> Dict[str, int]:
    return {str(key): int(value) for key, value in sorted(Counter(values).items())}


def evaluate_dataset(args: argparse.Namespace) -> Dict[str, object]:
    dataset_dir = Path(args.dataset_dir).expanduser().resolve()
    db_path = open_bag_db(dataset_dir)
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    cfg = SimpleLioConfig(
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

    keyframes: Deque[Keyframe] = deque()
    samples: List[Dict[str, object]] = []
    lio_path: List[Pose2] = []
    previous_pose: Optional[Pose2] = None
    previous_previous_pose: Optional[Pose2] = None
    first_cloud_stamp: Optional[float] = None
    error_sum = 0.0
    max_clouds = int(args.max_clouds)
    skipped_clouds = 0
    processed_clouds = 0
    icp_used_count = 0
    map_point_counts: List[int] = []

    for _, cloud in iter_cloud_rows(conn, ids[args.cloud_topic], start_stamp, end_stamp, args.cloud_stride):
        if max_clouds > 0 and processed_clouds >= max_clouds:
            break
        cloud_stamp = stamp_to_sec(cloud.header.stamp)
        truth_pose = interp_truth(cloud_stamp)
        if truth_pose is None:
            skipped_clouds += 1
            continue
        scan = extract_scan_2d(cloud, cfg)
        if len(scan) < cfg.icp_min_inliers:
            skipped_clouds += 1
            continue

        if first_cloud_stamp is None:
            first_cloud_stamp = cloud_stamp

        if previous_pose is None:
            pose = Pose2(
                stamp=cloud_stamp,
                x=truth_pose.x,
                y=truth_pose.y,
                yaw=truth_pose.yaw,
                vx=truth_pose.vx,
                vy=truth_pose.vy,
            )
            icp = IcpResult(pose=pose, used=False, inliers=0, rmse=None, median_error=None, iterations=0, reason="initial_gnss_ins_alignment")
        else:
            predicted = predict_pose(previous_pose, previous_previous_pose, cloud_stamp, imu, cfg)
            local_map = build_local_map(keyframes, predicted, cfg)
            map_point_counts.append(int(len(local_map)))
            icp = run_icp(scan, local_map, predicted, cfg)
            pose = icp.pose
            if icp.used:
                icp_used_count += 1
            if icp.used and len(scan) < cfg.sparse_scan_points:
                sparse_weight = cfg.sparse_icp_weight
                pose = blend_pose(predicted, pose, sparse_weight)
                icp.pose = pose
                icp.reason = f"{icp.reason}_sparse_scan_blend"

        if previous_pose is not None:
            pose, motion_clamped = clamp_motion(previous_pose, pose, cfg)
            if motion_clamped:
                icp.reason = f"{icp.reason}_motion_clamped"
                icp.pose = pose
            dt = pose.stamp - previous_pose.stamp
            if 1e-3 < dt <= cfg.max_frame_dt_sec:
                observed_vx = (pose.x - previous_pose.x) / dt
                observed_vy = (pose.y - previous_pose.y) / dt
                alpha = cfg.sparse_velocity_update_alpha if len(scan) < cfg.sparse_scan_points else cfg.velocity_update_alpha
                pose.vx = previous_pose.vx * (1.0 - alpha) + observed_vx * alpha
                pose.vy = previous_pose.vy * (1.0 - alpha) + observed_vy * alpha
            else:
                pose.vx = previous_pose.vx
                pose.vy = previous_pose.vy

        if should_add_keyframe(keyframes, pose, cfg):
            world_points = voxel_downsample(transform_points(scan, pose), cfg.map_voxel_m, cfg.max_keyframe_points)
            keyframes.append(Keyframe(stamp=pose.stamp, x=pose.x, y=pose.y, yaw=pose.yaw, points_world=world_points))
            while len(keyframes) > cfg.local_map_keyframes:
                keyframes.popleft()

        position_error = math.hypot(pose.x - truth_pose.x, pose.y - truth_pose.y)
        error_sum += position_error
        mean_error = error_sum / (len(samples) + 1)
        elapsed = cloud_stamp - first_cloud_stamp
        samples.append(
            {
                "t": round(elapsed, 6),
                "stamp": round(cloud_stamp, 6),
                "gnss": {
                    "x": round(truth_pose.x, 4),
                    "y": round(truth_pose.y, 4),
                    "yaw": round(truth_pose.yaw, 6),
                },
                "lio": {
                    "x": round(pose.x, 4),
                    "y": round(pose.y, 4),
                    "yaw": round(pose.yaw, 6),
                },
                "error_m": round(position_error, 4),
                "mean_error_m": round(mean_error, 4),
                "yaw_error_rad": round(abs(wrap_angle(pose.yaw - truth_pose.yaw)), 6),
                "icp": {
                    "used": bool(icp.used),
                    "inliers": int(icp.inliers),
                    "rmse_m": round_or_none(icp.rmse, 4),
                    "median_error_m": round_or_none(icp.median_error, 4),
                    "iterations": int(icp.iterations),
                    "reason": icp.reason,
                },
                "scan_points": int(len(scan)),
                "keyframes": int(len(keyframes)),
            }
        )
        lio_path.append(pose)
        previous_previous_pose = previous_pose
        previous_pose = pose
        processed_clouds += 1

    conn.close()

    errors = [float(sample["error_m"]) for sample in samples]
    yaw_errors = [float(sample["yaw_error_rad"]) for sample in samples]
    duration = float(samples[-1]["t"]) if samples else 0.0
    summary = {
        "sample_count": len(samples),
        "duration_sec": round(duration, 6),
        "processed_clouds": processed_clouds,
        "skipped_clouds": skipped_clouds,
        "icp_used_count": icp_used_count,
        "icp_used_ratio": round(icp_used_count / max(1, processed_clouds - 1), 6) if processed_clouds > 1 else 0.0,
        "position_error_m": finite_stats(errors),
        "yaw_error_rad": finite_stats(yaw_errors),
        "gnss_accuracy_horizon_m": finite_stats(truth.accuracy_horizon_m),
        "gnss_accuracy_yaw_deg": finite_stats(truth.accuracy_yaw_deg),
        "gnss_ins_status_counts": status_counts(truth.ins_status),
        "gnss_status_counts": status_counts(truth.gnss_status),
        "satellite_main": finite_stats(truth.satellite_main),
        "satellite_sub": finite_stats(truth.satellite_sub),
        "local_map_points": finite_stats(map_point_counts),
        "method": {
            "name": "simple_offline_lio_scan_to_local_map",
            "gnss_ins_usage": "GNSS/INS initializes the first LIO pose/yaw and is then used only for synchronization and final error evaluation.",
            "imu_usage": "IMU z angular rate predicts inter-scan yaw before LiDAR ICP.",
            "lidar_usage": "Non-ground LiDAR points are downsampled and matched against a recent local map with 3D nearest-neighbor, SE(2) point-to-point ICP.",
        },
        "config": cfg.__dict__,
        "runtime_sec": round(time.time() - started, 3),
    }

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
        "summary": summary,
        "samples": samples,
    }

    write_outputs(result, output_dir, Path(args.output_html).expanduser().resolve() if args.output_html else output_dir / "simple_lio_eval.html")
    return result


def write_outputs(result: Dict[str, object], output_dir: Path, output_html: Path) -> None:
    summary_path = output_dir / "summary.json"
    trace_path = output_dir / "lio_gnss_synced_trace.csv"
    data_path = output_dir / "lio_gnss_synced_trace.json"
    summary_path.write_text(json.dumps(result["summary"], indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    data_path.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    samples = result.get("samples", [])
    if samples:
        fieldnames = [
            "t",
            "stamp",
            "gnss_x",
            "gnss_y",
            "gnss_yaw",
            "lio_x",
            "lio_y",
            "lio_yaw",
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
            for sample in samples:
                icp = sample["icp"]
                writer.writerow(
                    {
                        "t": sample["t"],
                        "stamp": sample["stamp"],
                        "gnss_x": sample["gnss"]["x"],
                        "gnss_y": sample["gnss"]["y"],
                        "gnss_yaw": sample["gnss"]["yaw"],
                        "lio_x": sample["lio"]["x"],
                        "lio_y": sample["lio"]["y"],
                        "lio_yaw": sample["lio"]["yaw"],
                        "error_m": sample["error_m"],
                        "mean_error_m": sample["mean_error_m"],
                        "yaw_error_rad": sample["yaw_error_rad"],
                        "icp_used": icp["used"],
                        "icp_inliers": icp["inliers"],
                        "icp_rmse_m": icp["rmse_m"],
                        "icp_median_error_m": icp["median_error_m"],
                        "icp_iterations": icp["iterations"],
                        "icp_reason": icp["reason"],
                        "scan_points": sample["scan_points"],
                        "keyframes": sample["keyframes"],
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
    return HTML_TEMPLATE.replace("__LIO_EVAL_DATA__", payload)


HTML_TEMPLATE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RacingBrain LIO Evaluation</title>
<style>
html,body{margin:0;height:100%;overflow:hidden;background:#f5f6f1;color:#202427;font:13px/1.35 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
#stage{display:block;width:100vw;height:100vh;background:#f5f6f1;cursor:default}
#controls{position:fixed;left:0;right:0;bottom:0;height:46px;display:flex;align-items:center;gap:12px;padding:0 16px;background:rgba(245,246,241,.96);border-top:1px solid #cfd6ca}
button,select{height:28px;border:1px solid #9ca89c;background:#fff;color:#202427;border-radius:6px;padding:0 11px;font:12px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
#timeline{flex:1;min-width:120px}
#readout{min-width:190px;text-align:right;color:#596058}
</style>
</head>
<body>
<canvas id="stage"></canvas>
<div id="controls">
  <button id="play">Pause</button>
  <input id="timeline" type="range" min="0" max="0" value="0">
  <select id="speed"><option value="0.25">0.25x</option><option value="0.5">0.5x</option><option value="1" selected>1x</option><option value="2">2x</option><option value="4">4x</option><option value="8">8x</option></select>
  <button id="reset">Reset View</button>
  <span id="readout"></span>
</div>
<script id="lio-data" type="application/json">__LIO_EVAL_DATA__</script>
<script>
const data = JSON.parse(document.getElementById("lio-data").textContent);
const samples = data.samples || [];
const canvas = document.getElementById("stage");
const ctx = canvas.getContext("2d");
const playBtn = document.getElementById("play");
const timeline = document.getElementById("timeline");
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

timeline.max = String(Math.max(0, samples.length - 1));

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
  const side = Math.min(360, Math.max(286, innerWidth * 0.28));
  const mapH = Math.max(260, innerHeight - bottom - 190);
  const mapW = Math.max(330, innerWidth - side - pad * 3);
  return {
    map:{x:pad,y:pad,w:mapW,h:mapH},
    err:{x:pad,y:pad+mapH+12,w:mapW,h:innerHeight-bottom-(pad+mapH+12)-pad},
    panel:{x:pad+mapW+12,y:pad,w:side,h:innerHeight-bottom-pad*2}
  };
}

function allBounds(){
  let xs=[], ys=[];
  for(const s of samples){
    if(s.gnss){xs.push(s.gnss.x); ys.push(s.gnss.y);}
    if(s.lio){xs.push(s.lio.x); ys.push(s.lio.y);}
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

function drawPath(points, rect, color, upto, width){
  ctx.strokeStyle=color;
  ctx.lineWidth=width;
  ctx.beginPath();
  let started=false;
  const n = Math.min(points.length, upto + 1);
  for(let i=0;i<n;i++){
    const p = points[i];
    const s = worldToScreen(p, rect);
    if(!started){ctx.moveTo(s.x,s.y); started=true;} else {ctx.lineTo(s.x,s.y);}
  }
  ctx.stroke();
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

function niceGrid(raw){
  const p = Math.pow(10, Math.floor(Math.log10(Math.max(raw, 1e-6))));
  const n = raw / p;
  if(n < 2) return p;
  if(n < 5) return 2*p;
  return 5*p;
}

function drawMap(rect){
  panel(rect, "Global trajectory map");
  ctx.save();
  clip({x:rect.x+1,y:rect.y+30,w:rect.w-2,h:rect.h-31});
  drawGrid(rect);
  const gnss = samples.map(s => s.gnss);
  const lio = samples.map(s => s.lio);
  drawPath(gnss, rect, "#148a5a", idx, 2.6);
  drawPath(lio, rect, "#315fc7", idx, 2.1);
  const cur = samples[idx];
  if(cur){
    for(const item of [{p:cur.gnss,c:"#148a5a"},{p:cur.lio,c:"#315fc7"}]){
      const q = worldToScreen(item.p, rect);
      ctx.fillStyle=item.c;
      ctx.beginPath(); ctx.arc(q.x,q.y,5,0,Math.PI*2); ctx.fill();
      ctx.strokeStyle="#fff"; ctx.lineWidth=2; ctx.stroke();
    }
    ctx.strokeStyle="#d45b31";
    ctx.lineWidth=1.5;
    const a=worldToScreen(cur.gnss,rect), b=worldToScreen(cur.lio,rect);
    ctx.beginPath(); ctx.moveTo(a.x,a.y); ctx.lineTo(b.x,b.y); ctx.stroke();
  }
  ctx.restore();
  legend(rect.x+12, rect.y+42, [["#148a5a","GNSS/INS truth"],["#315fc7","LIO"],["#d45b31","position error"]]);
}

function legend(x,y,items){
  ctx.font="12px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace";
  let offset=0;
  for(const [color,label] of items){
    ctx.fillStyle=color; ctx.fillRect(x+offset,y-8,18,3);
    ctx.fillStyle="#596058"; ctx.fillText(label,x+offset+24,y-4);
    offset += ctx.measureText(label).width + 52;
  }
}

function drawErrors(rect){
  panel(rect, "Position error");
  const plot = {x:rect.x+44,y:rect.y+36,w:rect.w-62,h:rect.h-54};
  ctx.strokeStyle="#d6ddd2"; ctx.strokeRect(plot.x+.5,plot.y+.5,plot.w-1,plot.h-1);
  const maxErr = Math.max(1, ...samples.map(s => Math.max(s.error_m || 0, s.mean_error_m || 0)));
  function px(i){ return plot.x + (samples.length <= 1 ? 0 : i/(samples.length-1))*plot.w; }
  function py(v){ return plot.y + plot.h - (v/maxErr)*plot.h; }
  function curve(key,color,width){
    ctx.strokeStyle=color; ctx.lineWidth=width; ctx.beginPath();
    for(let i=0;i<samples.length;i++){
      const x=px(i), y=py(samples[i][key] || 0);
      if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }
  curve("error_m","#d45b31",1.8);
  curve("mean_error_m","#315fc7",1.8);
  const x=px(idx);
  ctx.strokeStyle="#202427"; ctx.lineWidth=1; ctx.beginPath(); ctx.moveTo(x,plot.y); ctx.lineTo(x,plot.y+plot.h); ctx.stroke();
  ctx.fillStyle="#596058"; ctx.font="11px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace";
  ctx.fillText(maxErr.toFixed(1)+" m", rect.x+8, plot.y+4);
  ctx.fillText("0", rect.x+28, plot.y+plot.h);
  legend(plot.x+8, rect.y+rect.h-11, [["#d45b31","instant"],["#315fc7","cumulative mean"]]);
}

function fmt(v,d=3){
  if(v === null || v === undefined || !Number.isFinite(Number(v))) return "n/a";
  return Number(v).toFixed(d);
}

function drawPanel(rect){
  panel(rect, "Values");
  const s = samples[idx] || {};
  const sum = data.summary || {};
  const pos = (sum.position_error_m || {});
  const icp = s.icp || {};
  const rows = [
    ["Samples", String(samples.length)],
    ["Elapsed", fmt(s.t,2)+" s"],
    ["GNSS x/y", fmt(s.gnss && s.gnss.x,2)+", "+fmt(s.gnss && s.gnss.y,2)],
    ["LIO x/y", fmt(s.lio && s.lio.x,2)+", "+fmt(s.lio && s.lio.y,2)],
    ["Error", fmt(s.error_m,3)+" m"],
    ["Mean error", fmt(s.mean_error_m,3)+" m"],
    ["Max error", fmt(pos.max,3)+" m"],
    ["RMSE", fmt(pos.rmse,3)+" m"],
    ["ICP used", String(icp.used === true)],
    ["ICP inliers", String(icp.inliers ?? "n/a")],
    ["ICP RMSE", fmt(icp.rmse_m,3)+" m"],
    ["Keyframes", String(s.keyframes ?? "n/a")],
    ["INS status", JSON.stringify(sum.gnss_ins_status_counts || {})],
    ["GNSS status", JSON.stringify(sum.gnss_status_counts || {})]
  ];
  ctx.font="12px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace";
  let y = rect.y + 50;
  for(const [k,v] of rows){
    ctx.fillStyle="#6a7168"; ctx.fillText(k, rect.x+14, y);
    ctx.fillStyle="#202427"; ctx.fillText(v, rect.x+128, y);
    y += 24;
  }
  ctx.fillStyle="#6a7168";
  const note = (sum.method && sum.method.gnss_ins_usage) || "";
  wrapText(note, rect.x+14, rect.y+rect.h-76, rect.w-28, 16);
}

function wrapText(text,x,y,w,lineH){
  const words = String(text).split(/\s+/);
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
  drawMap(r.map);
  drawErrors(r.err);
  drawPanel(r.panel);
  const cur = samples[idx];
  readout.textContent = cur ? `${idx+1}/${samples.length}  ${fmt(cur.t,2)}s  ${fmt(cur.error_m,2)}m` : "0/0";
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

playBtn.onclick = () => {
  playing = !playing;
  if(playing && idx >= samples.length-1) idx = 0;
  playBtn.textContent = playing ? "Pause" : "Play";
};
timeline.oninput = () => { idx = Number(timeline.value); playing = false; playBtn.textContent = "Play"; draw(); };
resetBtn.onclick = () => { resetView(); draw(); };

canvas.addEventListener("mousedown", ev => {
  const r = layout().map;
  if(ev.clientX >= r.x && ev.clientX <= r.x+r.w && ev.clientY >= r.y && ev.clientY <= r.y+r.h){
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
  const r = layout().map;
  if(!(ev.clientX >= r.x && ev.clientX <= r.x+r.w && ev.clientY >= r.y && ev.clientY <= r.y+r.h)) return;
  ev.preventDefault();
  const before = screenToWorld(ev.clientX, ev.clientY, r);
  const factor = ev.deltaY < 0 ? 1.15 : 1/1.15;
  const minScale = Math.max(0.001, view.fitScale * 0.04);
  const maxScale = Math.max(400, view.fitScale * 80);
  view.scale = Math.min(maxScale, Math.max(minScale, view.scale * factor));
  const after = screenToWorld(ev.clientX, ev.clientY, r);
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
    parser = argparse.ArgumentParser(description="Run a simple offline LiDAR-IMU odometry evaluation against GNSS/INS truth.")
    parser.add_argument("--dataset-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--output-html", default="")
    parser.add_argument("--cloud-topic", default="/lidar_points")
    parser.add_argument("--imu-topic", default="/imu")
    parser.add_argument("--gnss-topic", default="/gongji_gnss_ins_64")
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
    summary = result["summary"]
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
