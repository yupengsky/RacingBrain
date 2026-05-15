from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


def clamp(value: float, lower: float = 0.0, upper: float = 1.0) -> float:
    return max(lower, min(upper, float(value)))


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def angle_error(a: float, b: float) -> float:
    return abs(wrap_angle(a - b))


def blend_angle(a: float, b: float, weight_b: float) -> float:
    weight_b = clamp(weight_b)
    x = (1.0 - weight_b) * math.cos(a) + weight_b * math.cos(b)
    y = (1.0 - weight_b) * math.sin(a) + weight_b * math.sin(b)
    if abs(x) < 1e-12 and abs(y) < 1e-12:
        return wrap_angle(a)
    return math.atan2(y, x)


@dataclass
class Pose2D:
    stamp: float
    x: float
    y: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    frame_id: str = "map"
    source: str = ""

    def distance_to(self, other: "Pose2D") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


@dataclass
class SourceQuality:
    source: str
    state: str
    score: float
    age_sec: Optional[float]
    reasons: List[str] = field(default_factory=list)

    @property
    def usable(self) -> bool:
        return self.score > 0.05 and self.state not in {"missing", "stale"}


@dataclass
class SourceSample:
    pose: Pose2D
    covariance_xy: Optional[float] = None
    covariance_yaw: Optional[float] = None
    accuracy_xy: Optional[float] = None
    accuracy_yaw: Optional[float] = None
    status: Optional[int] = None


@dataclass
class PoseAlignment:
    yaw_offset: float
    tx: float
    ty: float
    initialized_at: float

    def transform(self, pose: Pose2D) -> Pose2D:
        c = math.cos(self.yaw_offset)
        s = math.sin(self.yaw_offset)
        x = c * pose.x - s * pose.y + self.tx
        y = s * pose.x + c * pose.y + self.ty
        vx = c * pose.vx - s * pose.vy
        vy = s * pose.vx + c * pose.vy
        return Pose2D(
            stamp=pose.stamp,
            x=x,
            y=y,
            yaw=wrap_angle(pose.yaw + self.yaw_offset),
            vx=vx,
            vy=vy,
            yaw_rate=pose.yaw_rate,
            frame_id="map",
            source="lio_aligned",
        )


@dataclass
class PoseJudgeConfig:
    stale_timeout_sec: float = 0.35
    gnss_accuracy_good_m: float = 0.25
    gnss_accuracy_warn_m: float = 1.0
    gnss_accuracy_bad_m: float = 2.5
    lio_cov_good_m2: float = 0.08
    lio_cov_warn_m2: float = 0.55
    lio_cov_bad_m2: float = 2.0
    cross_position_warn_m: float = 0.75
    cross_position_reject_m: float = 2.0
    cross_yaw_warn_rad: float = 0.18
    cross_yaw_reject_rad: float = 0.55
    max_jump_m: float = 4.0
    max_yaw_jump_rad: float = 0.7
    prefer_gnss_margin: float = 0.08
    fused_min_score: float = 0.55
    fusion_enabled: bool = True
    alignment_update_alpha: float = 0.03


@dataclass
class PoseDecision:
    state: str
    source: str
    pose: Optional[Pose2D]
    reasons: List[str]
    qualities: Dict[str, SourceQuality]
    cross_position_error_m: Optional[float] = None
    cross_yaw_error_rad: Optional[float] = None
    alignment_initialized: bool = False

    def as_dict(self) -> Dict[str, object]:
        return {
            "state": self.state,
            "source": self.source,
            "pose": None
            if self.pose is None
            else {
                "stamp": self.pose.stamp,
                "x": self.pose.x,
                "y": self.pose.y,
                "yaw": self.pose.yaw,
                "vx": self.pose.vx,
                "vy": self.pose.vy,
                "yaw_rate": self.pose.yaw_rate,
                "frame_id": self.pose.frame_id,
                "source": self.pose.source,
            },
            "reasons": list(self.reasons),
            "qualities": {
                key: {
                    "state": value.state,
                    "score": value.score,
                    "age_sec": value.age_sec,
                    "reasons": list(value.reasons),
                }
                for key, value in self.qualities.items()
            },
            "cross_position_error_m": self.cross_position_error_m,
            "cross_yaw_error_rad": self.cross_yaw_error_rad,
            "alignment_initialized": self.alignment_initialized,
        }


class MultiSourcePoseJudge:
    """Judge GNSS/INS and LIO poses before mapping consumes vehicle pose.

    The mapping node needs a trusted, time-aligned vehicle pose rather than a
    specific sensor packet. This class keeps that boundary explicit: GNSS/INS
    is the absolute anchor, LIO is the locally continuous source, and the judge
    chooses or blends them based on freshness, covariance and cross-source
    consistency.
    """

    def __init__(self, config: Optional[PoseJudgeConfig] = None) -> None:
        self.config = config or PoseJudgeConfig()
        self.gnss: Optional[SourceSample] = None
        self.lio_raw: Optional[SourceSample] = None
        self.alignment: Optional[PoseAlignment] = None
        self.last_by_source: Dict[str, Pose2D] = {}
        self.previous_by_source: Dict[str, Pose2D] = {}

    def update_gnss(
        self,
        pose: Pose2D,
        *,
        accuracy_xy: Optional[float] = None,
        accuracy_yaw: Optional[float] = None,
        status: Optional[int] = None,
    ) -> None:
        pose.source = "gnss_ins"
        if self.gnss is not None:
            self.previous_by_source["gnss_ins"] = self.gnss.pose
        self.gnss = SourceSample(pose=pose, accuracy_xy=accuracy_xy, accuracy_yaw=accuracy_yaw, status=status)

    def update_lio(
        self,
        pose: Pose2D,
        *,
        covariance_xy: Optional[float] = None,
        covariance_yaw: Optional[float] = None,
    ) -> None:
        pose.source = "lio"
        if self.lio_raw is not None:
            self.previous_by_source["lio"] = self.lio_raw.pose
        self.lio_raw = SourceSample(pose=pose, covariance_xy=covariance_xy, covariance_yaw=covariance_yaw)

    def decide(self, now: Optional[float] = None) -> PoseDecision:
        now = self._infer_now(now)
        gnss_q = self._quality("gnss_ins", self.gnss, now)
        lio_q_raw = self._quality("lio", self.lio_raw, now)
        qualities = {"gnss_ins": gnss_q, "lio": lio_q_raw}

        self._maybe_initialize_or_update_alignment(now, gnss_q, lio_q_raw)
        lio_aligned = self._aligned_lio_sample()
        lio_q = self._quality("lio", lio_aligned, now)
        qualities["lio"] = lio_q

        candidates: Dict[str, Tuple[SourceSample, SourceQuality]] = {}
        if self.gnss is not None and gnss_q.usable:
            candidates["gnss_ins"] = (self.gnss, gnss_q)
        if lio_aligned is not None and lio_q.usable:
            candidates["lio"] = (lio_aligned, lio_q)

        if not candidates:
            state = "aligning" if lio_q_raw.usable and self.alignment is None else "missing"
            reasons = ["no_usable_pose_source"]
            if state == "aligning":
                reasons.append("lio_waiting_for_gnss_alignment")
            return PoseDecision(state, "none", None, reasons, qualities, alignment_initialized=self.alignment is not None)

        if "gnss_ins" not in candidates:
            pose = self._remember_and_return("lio", candidates["lio"][0].pose)
            return PoseDecision(
                "degraded",
                "lio",
                pose,
                ["gnss_unusable_using_lio"],
                qualities,
                alignment_initialized=self.alignment is not None,
            )

        if "lio" not in candidates:
            pose = self._remember_and_return("gnss_ins", candidates["gnss_ins"][0].pose)
            return PoseDecision(
                "degraded",
                "gnss_ins",
                pose,
                ["lio_unusable_using_gnss"],
                qualities,
                alignment_initialized=self.alignment is not None,
            )

        gnss_pose = candidates["gnss_ins"][0].pose
        lio_pose = candidates["lio"][0].pose
        pos_err = gnss_pose.distance_to(lio_pose)
        yaw_err = angle_error(gnss_pose.yaw, lio_pose.yaw)
        reasons: List[str] = []
        state = "nominal"

        if pos_err >= self.config.cross_position_reject_m or yaw_err >= self.config.cross_yaw_reject_rad:
            state = "conflict"
            reasons.append("gnss_lio_consistency_reject")
        elif pos_err >= self.config.cross_position_warn_m or yaw_err >= self.config.cross_yaw_warn_rad:
            state = "warn"
            reasons.append("gnss_lio_consistency_warn")

        gnss_score = candidates["gnss_ins"][1].score
        lio_score = candidates["lio"][1].score
        if self.config.fusion_enabled and state == "nominal" and min(gnss_score, lio_score) >= self.config.fused_min_score:
            pose = self._fuse(gnss_pose, lio_pose, gnss_score, lio_score)
            return PoseDecision(
                "nominal",
                "fused",
                self._remember_and_return("fused", pose),
                ["gnss_lio_consistent_fused"],
                qualities,
                pos_err,
                yaw_err,
                self.alignment is not None,
            )

        if gnss_score + self.config.prefer_gnss_margin >= lio_score:
            chosen = "gnss_ins"
        else:
            chosen = "lio"
        if not reasons:
            reasons.append(f"{chosen}_higher_quality")
        pose = self._remember_and_return(chosen, candidates[chosen][0].pose)
        return PoseDecision(
            state,
            chosen,
            pose,
            reasons,
            qualities,
            pos_err,
            yaw_err,
            self.alignment is not None,
        )

    def _infer_now(self, now: Optional[float]) -> float:
        if now is not None:
            return float(now)
        stamps = []
        if self.gnss is not None:
            stamps.append(self.gnss.pose.stamp)
        if self.lio_raw is not None:
            stamps.append(self.lio_raw.pose.stamp)
        return max(stamps) if stamps else 0.0

    def _quality(self, source: str, sample: Optional[SourceSample], now: float) -> SourceQuality:
        if sample is None:
            return SourceQuality(source, "missing", 0.0, None, ["missing"])

        age = max(0.0, now - sample.pose.stamp)
        reasons: List[str] = []
        score = 1.0
        state = "ok"

        if age > self.config.stale_timeout_sec:
            return SourceQuality(source, "stale", 0.0, age, ["stale"])

        if source == "gnss_ins":
            accuracy = sample.accuracy_xy
            if accuracy is not None:
                if accuracy >= self.config.gnss_accuracy_bad_m:
                    score *= 0.05
                    state = "degraded"
                    reasons.append("gnss_accuracy_bad")
                elif accuracy >= self.config.gnss_accuracy_warn_m:
                    score *= 0.35
                    state = "degraded"
                    reasons.append("gnss_accuracy_warn")
                elif accuracy > self.config.gnss_accuracy_good_m:
                    score *= 0.75
                    reasons.append("gnss_accuracy_moderate")
        else:
            covariance = sample.covariance_xy
            if self.alignment is None:
                score *= 0.35
                state = "degraded"
                reasons.append("lio_not_map_aligned")
            if covariance is not None:
                if covariance >= self.config.lio_cov_bad_m2:
                    score *= 0.05
                    state = "degraded"
                    reasons.append("lio_covariance_bad")
                elif covariance >= self.config.lio_cov_warn_m2:
                    score *= 0.45
                    state = "degraded"
                    reasons.append("lio_covariance_warn")
                elif covariance > self.config.lio_cov_good_m2:
                    score *= 0.80
                    reasons.append("lio_covariance_moderate")

        previous = self.previous_by_source.get(source)
        if previous is not None:
            jump = sample.pose.distance_to(previous)
            yaw_jump = angle_error(sample.pose.yaw, previous.yaw)
            if jump > self.config.max_jump_m:
                score *= 0.20
                state = "degraded"
                reasons.append("pose_jump")
            if yaw_jump > self.config.max_yaw_jump_rad:
                score *= 0.30
                state = "degraded"
                reasons.append("yaw_jump")

        if not reasons:
            reasons.append("fresh")
        return SourceQuality(source, state, clamp(score), age, reasons)

    def _maybe_initialize_or_update_alignment(self, now: float, gnss_q: SourceQuality, lio_q: SourceQuality) -> None:
        if self.gnss is None or self.lio_raw is None:
            return
        if not gnss_q.usable or not lio_q.usable:
            return
        if abs(self.gnss.pose.stamp - self.lio_raw.pose.stamp) > self.config.stale_timeout_sec:
            return

        yaw_offset = wrap_angle(self.gnss.pose.yaw - self.lio_raw.pose.yaw)
        c = math.cos(yaw_offset)
        s = math.sin(yaw_offset)
        lio_x = c * self.lio_raw.pose.x - s * self.lio_raw.pose.y
        lio_y = s * self.lio_raw.pose.x + c * self.lio_raw.pose.y
        tx = self.gnss.pose.x - lio_x
        ty = self.gnss.pose.y - lio_y
        if self.alignment is None:
            self.alignment = PoseAlignment(yaw_offset=yaw_offset, tx=tx, ty=ty, initialized_at=now)
            return

        if gnss_q.score < 0.70 or lio_q.score < 0.70:
            return

        alpha = clamp(self.config.alignment_update_alpha)
        self.alignment.yaw_offset = blend_angle(self.alignment.yaw_offset, yaw_offset, alpha)
        self.alignment.tx = (1.0 - alpha) * self.alignment.tx + alpha * tx
        self.alignment.ty = (1.0 - alpha) * self.alignment.ty + alpha * ty

    def _aligned_lio_sample(self) -> Optional[SourceSample]:
        if self.lio_raw is None or self.alignment is None:
            return None
        return SourceSample(
            pose=self.alignment.transform(self.lio_raw.pose),
            covariance_xy=self.lio_raw.covariance_xy,
            covariance_yaw=self.lio_raw.covariance_yaw,
            status=self.lio_raw.status,
        )

    def _fuse(self, gnss_pose: Pose2D, lio_pose: Pose2D, gnss_score: float, lio_score: float) -> Pose2D:
        total = max(gnss_score + lio_score, 1e-6)
        lio_weight = lio_score / total
        return Pose2D(
            stamp=max(gnss_pose.stamp, lio_pose.stamp),
            x=(1.0 - lio_weight) * gnss_pose.x + lio_weight * lio_pose.x,
            y=(1.0 - lio_weight) * gnss_pose.y + lio_weight * lio_pose.y,
            yaw=blend_angle(gnss_pose.yaw, lio_pose.yaw, lio_weight),
            vx=(1.0 - lio_weight) * gnss_pose.vx + lio_weight * lio_pose.vx,
            vy=(1.0 - lio_weight) * gnss_pose.vy + lio_weight * lio_pose.vy,
            yaw_rate=(1.0 - lio_weight) * gnss_pose.yaw_rate + lio_weight * lio_pose.yaw_rate,
            frame_id="map",
            source="fused",
        )

    def _remember_and_return(self, source: str, pose: Pose2D) -> Pose2D:
        self.last_by_source[source] = pose
        return pose
