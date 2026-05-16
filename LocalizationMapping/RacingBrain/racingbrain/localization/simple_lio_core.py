from __future__ import annotations

import bisect
import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Protocol, Tuple

import numpy as np
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from racingbrain.localization.pose_sources import wrap_angle


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
    lidar_min_range_m: float = 2.0
    lidar_max_range_m: float = 90.0
    z_min_m: float = -0.5
    z_max_m: float = 4.0
    icp_z_weight: float = 0.7
    scan_voxel_m: float = 0.35
    map_voxel_m: float = 0.35
    max_scan_points: int = 6500
    max_keyframe_points: int = 4500
    local_map_keyframes: int = 35
    keyframe_dist_m: float = 0.45
    keyframe_yaw_rad: float = 0.08
    keyframe_time_sec: float = 1.2
    icp_iterations: int = 10
    icp_max_correspondence_m: float = 2.0
    icp_trim_fraction: float = 0.82
    icp_min_inliers: int = 80
    icp_max_rmse_m: float = 1.5
    icp_max_translation_step_m: float = 0.45
    icp_max_yaw_step_rad: float = 0.08
    icp_max_pose_correction_m: float = 0.55
    icp_max_pose_correction_yaw_rad: float = 0.06
    imu_gyro_scale: float = 0.04348764102608839
    max_frame_dt_sec: float = 0.35
    max_speed_mps: float = 4.0
    max_yaw_rate_radps: float = 1.0
    sparse_scan_points: int = 1200
    sparse_icp_weight: float = 0.25
    velocity_update_alpha: float = 0.45
    sparse_velocity_update_alpha: float = 0.05


class YawDeltaSource(Protocol):
    def yaw_delta(self, start: float, end: float) -> float:
        ...


class OnlineImuYawBuffer:
    def __init__(self, *, gyro_scale: float, max_age_sec: float = 30.0) -> None:
        self.gyro_scale = float(gyro_scale)
        self.max_age_sec = float(max_age_sec)
        self.stamps: Deque[float] = deque()
        self.yaw_rates: Deque[float] = deque()
        self.yaw_integral: Deque[float] = deque()

    def add(self, stamp: float, raw_yaw_rate: float) -> None:
        if stamp <= 0.0:
            return
        yaw_rate = float(raw_yaw_rate) * self.gyro_scale
        if not math.isfinite(yaw_rate):
            return
        if self.stamps and stamp <= self.stamps[-1]:
            self._insert_sorted(stamp, yaw_rate)
        else:
            integral = 0.0
            if self.stamps:
                dt = max(0.0, min(0.2, stamp - self.stamps[-1]))
                integral = self.yaw_integral[-1] + 0.5 * (self.yaw_rates[-1] + yaw_rate) * dt
            self.stamps.append(float(stamp))
            self.yaw_rates.append(yaw_rate)
            self.yaw_integral.append(integral)
        self._trim(float(stamp) - self.max_age_sec)

    def yaw_delta(self, start: float, end: float) -> float:
        if len(self.stamps) < 2 or end <= start:
            return 0.0
        stamps = np.asarray(self.stamps, dtype=np.float64)
        integral = np.asarray(self.yaw_integral, dtype=np.float64)
        lo = float(np.clip(start, stamps[0], stamps[-1]))
        hi = float(np.clip(end, stamps[0], stamps[-1]))
        if hi <= lo:
            return 0.0
        return float(np.interp(hi, stamps, integral) - np.interp(lo, stamps, integral))

    def _insert_sorted(self, stamp: float, yaw_rate: float) -> None:
        stamps = list(self.stamps)
        rates = list(self.yaw_rates)
        index = bisect.bisect_left(stamps, stamp)
        if index < len(stamps) and abs(stamps[index] - stamp) < 1e-9:
            rates[index] = yaw_rate
        else:
            stamps.insert(index, stamp)
            rates.insert(index, yaw_rate)
        self.stamps = deque(stamps)
        self.yaw_rates = deque(rates)
        self._rebuild_integral()

    def _rebuild_integral(self) -> None:
        integrals = [0.0]
        stamps = list(self.stamps)
        rates = list(self.yaw_rates)
        for index in range(1, len(stamps)):
            dt = max(0.0, min(0.2, stamps[index] - stamps[index - 1]))
            integrals.append(integrals[-1] + 0.5 * (rates[index - 1] + rates[index]) * dt)
        self.yaw_integral = deque(integrals)

    def _trim(self, cutoff: float) -> None:
        while len(self.stamps) > 2 and self.stamps[0] < cutoff:
            self.stamps.popleft()
            self.yaw_rates.popleft()
            self.yaw_integral.popleft()
        if self.yaw_integral:
            offset = self.yaw_integral[0]
            self.yaw_integral = deque(value - offset for value in self.yaw_integral)


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


def build_local_map(keyframes: Deque[Keyframe], cfg: SimpleLioConfig) -> np.ndarray:
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


def predict_pose(previous: Pose2, previous_previous: Optional[Pose2], stamp: float, imu: YawDeltaSource, cfg: SimpleLioConfig) -> Pose2:
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

