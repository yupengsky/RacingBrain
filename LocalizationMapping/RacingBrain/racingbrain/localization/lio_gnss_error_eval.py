from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional

from racingbrain.localization.pose_sources import Pose2D, PoseAlignment, angle_error, wrap_angle


@dataclass
class ErrorSample:
    stamp: float
    gnss_stamp: float
    lio_stamp: float
    time_offset_sec: float
    gnss_x: float
    gnss_y: float
    gnss_yaw: float
    lio_x: float
    lio_y: float
    lio_yaw: float
    position_error_m: float
    yaw_error_rad: float
    gnss_accuracy_m: Optional[float]
    lio_covariance_xy: Optional[float]

    def as_row(self) -> Dict[str, object]:
        return {
            "stamp": round(self.stamp, 6),
            "gnss_stamp": round(self.gnss_stamp, 6),
            "lio_stamp": round(self.lio_stamp, 6),
            "time_offset_sec": round(self.time_offset_sec, 6),
            "gnss_x": round(self.gnss_x, 4),
            "gnss_y": round(self.gnss_y, 4),
            "gnss_yaw_rad": round(self.gnss_yaw, 6),
            "lio_x": round(self.lio_x, 4),
            "lio_y": round(self.lio_y, 4),
            "lio_yaw_rad": round(self.lio_yaw, 6),
            "position_error_m": round(self.position_error_m, 4),
            "yaw_error_rad": round(self.yaw_error_rad, 6),
            "gnss_accuracy_m": "" if self.gnss_accuracy_m is None else round(self.gnss_accuracy_m, 4),
            "lio_covariance_xy": "" if self.lio_covariance_xy is None else round(self.lio_covariance_xy, 6),
        }


@dataclass
class GnssSample:
    pose: Pose2D
    accuracy_xy: Optional[float]


def percentile(values: List[float], pct: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil((pct / 100.0) * len(ordered)) - 1))
    return ordered[index]


class LioGnssErrorEvaluator:
    def __init__(
        self,
        *,
        sync_tolerance_sec: float = 0.08,
        gnss_queue_sec: float = 5.0,
        alignment_mode: str = "first_pair",
    ) -> None:
        self.sync_tolerance_sec = float(sync_tolerance_sec)
        self.gnss_queue_sec = float(gnss_queue_sec)
        self.alignment_mode = alignment_mode
        self.gnss_queue: Deque[GnssSample] = deque()
        self.alignment: Optional[PoseAlignment] = None
        self.samples: List[ErrorSample] = []

    @property
    def aligned(self) -> bool:
        return self.alignment is not None or self.alignment_mode == "none"

    def update_gnss(self, sample: GnssSample) -> None:
        self.gnss_queue.append(sample)
        cutoff = sample.pose.stamp - self.gnss_queue_sec
        while self.gnss_queue and self.gnss_queue[0].pose.stamp < cutoff:
            self.gnss_queue.popleft()

    def update_lio(self, lio_pose: Pose2D, covariance_xy: Optional[float] = None) -> Optional[ErrorSample]:
        gnss = self.nearest_gnss(lio_pose.stamp)
        if gnss is None:
            return None
        lio_aligned = self.align_lio(lio_pose, gnss.pose)
        if lio_aligned is None:
            return None
        position_error = gnss.pose.distance_to(lio_aligned)
        yaw_error = angle_error(gnss.pose.yaw, lio_aligned.yaw)
        sample = ErrorSample(
            stamp=max(gnss.pose.stamp, lio_pose.stamp),
            gnss_stamp=gnss.pose.stamp,
            lio_stamp=lio_pose.stamp,
            time_offset_sec=lio_pose.stamp - gnss.pose.stamp,
            gnss_x=gnss.pose.x,
            gnss_y=gnss.pose.y,
            gnss_yaw=gnss.pose.yaw,
            lio_x=lio_aligned.x,
            lio_y=lio_aligned.y,
            lio_yaw=lio_aligned.yaw,
            position_error_m=position_error,
            yaw_error_rad=yaw_error,
            gnss_accuracy_m=gnss.accuracy_xy,
            lio_covariance_xy=covariance_xy,
        )
        self.samples.append(sample)
        return sample

    def nearest_gnss(self, stamp: float) -> Optional[GnssSample]:
        if not self.gnss_queue:
            return None
        best = min(self.gnss_queue, key=lambda sample: abs(sample.pose.stamp - stamp))
        if abs(best.pose.stamp - stamp) > self.sync_tolerance_sec:
            return None
        return best

    def align_lio(self, lio_pose: Pose2D, gnss_pose: Pose2D) -> Optional[Pose2D]:
        if self.alignment_mode == "none":
            return Pose2D(
                stamp=lio_pose.stamp,
                x=lio_pose.x,
                y=lio_pose.y,
                yaw=lio_pose.yaw,
                vx=lio_pose.vx,
                vy=lio_pose.vy,
                yaw_rate=lio_pose.yaw_rate,
                frame_id="map",
                source="lio",
            )
        if self.alignment is None:
            yaw_offset = wrap_angle(gnss_pose.yaw - lio_pose.yaw)
            c = math.cos(yaw_offset)
            s = math.sin(yaw_offset)
            lio_x = c * lio_pose.x - s * lio_pose.y
            lio_y = s * lio_pose.x + c * lio_pose.y
            self.alignment = PoseAlignment(
                yaw_offset=yaw_offset,
                tx=gnss_pose.x - lio_x,
                ty=gnss_pose.y - lio_y,
                initialized_at=max(gnss_pose.stamp, lio_pose.stamp),
            )
        return self.alignment.transform(lio_pose)

    def summary(self, *, position_warn_m: float, yaw_warn_rad: float) -> Dict[str, object]:
        position_errors = [sample.position_error_m for sample in self.samples]
        yaw_errors = [sample.yaw_error_rad for sample in self.samples]
        time_offsets = [abs(sample.time_offset_sec) for sample in self.samples]

        def stats(values: List[float]) -> Dict[str, Optional[float]]:
            if not values:
                return {"mean": None, "median": None, "p95": None, "max": None}
            return {
                "mean": round(statistics.fmean(values), 6),
                "median": round(statistics.median(values), 6),
                "p95": round(percentile(values, 95.0) or 0.0, 6),
                "max": round(max(values), 6),
            }

        return {
            "sample_count": len(self.samples),
            "alignment_initialized": self.alignment is not None,
            "alignment_mode": self.alignment_mode,
            "position_error_m": stats(position_errors),
            "yaw_error_rad": stats(yaw_errors),
            "abs_time_offset_sec": stats(time_offsets),
            "position_warn_m": position_warn_m,
            "yaw_warn_rad": yaw_warn_rad,
            "position_warn_count": sum(1 for value in position_errors if value >= position_warn_m),
            "yaw_warn_count": sum(1 for value in yaw_errors if value >= yaw_warn_rad),
        }
