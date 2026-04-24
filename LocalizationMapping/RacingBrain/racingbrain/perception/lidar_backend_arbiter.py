from __future__ import annotations

import json
import math
import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from test_cone_segmentation.msg import ThreeDConeArray


def now_sec(node: Node) -> float:
    return float(node.get_clock().now().nanoseconds) * 1e-9


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def as_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def mean(values: List[float]) -> Optional[float]:
    if not values:
        return None
    return sum(values) / float(len(values))


def append_unique(values: List[str], item: str) -> None:
    if item and item not in values:
        values.append(item)


class BackendWindow:
    def __init__(self, maxlen: int) -> None:
        self.last_msg: Optional[ThreeDConeArray] = None
        self.last_msg_wall: Optional[float] = None
        self.last_metrics: Optional[Dict[str, Any]] = None
        self.last_metrics_wall: Optional[float] = None
        self.cone_counts: Deque[int] = deque(maxlen=maxlen)
        self.total_ms: Deque[float] = deque(maxlen=maxlen)

    def push_msg(self, msg: ThreeDConeArray, wall_time: float) -> None:
        self.last_msg = msg
        self.last_msg_wall = wall_time
        self.cone_counts.append(len(msg.cones))

    def push_metrics(self, payload: Dict[str, Any], wall_time: float) -> None:
        self.last_metrics = payload
        self.last_metrics_wall = wall_time
        cone_count = as_float(payload.get("cone_count"))
        if cone_count is not None:
            self.cone_counts.append(int(cone_count))
        total_ms = as_float(payload.get("total_ms"))
        if total_ms is not None:
            self.total_ms.append(total_ms)

    def age_sec(self, wall_time: float) -> Optional[float]:
        candidates = [value for value in (self.last_msg_wall, self.last_metrics_wall) if value is not None]
        if not candidates:
            return None
        return wall_time - max(candidates)

    def available(self, wall_time: float, stale_timeout_sec: float) -> bool:
        age = self.age_sec(wall_time)
        return age is not None and age <= stale_timeout_sec

    def empty_ratio(self) -> Optional[float]:
        if not self.cone_counts:
            return None
        empty = sum(1 for count in self.cone_counts if count == 0)
        return empty / float(len(self.cone_counts))

    def mean_cone_count(self) -> Optional[float]:
        return mean([float(value) for value in self.cone_counts])

    def mean_total_ms(self) -> Optional[float]:
        return mean(list(self.total_ms))


class LidarBackendArbiter(Node):
    """Selects the final LiDAR cone stream and explains perception failures."""

    def __init__(self) -> None:
        super().__init__("lidar_backend_arbiter")

        self.declare_parameter("mode", "auto")
        self.declare_parameter("preferred_backend", "pointpillars")
        self.declare_parameter("fallback_backend", "cluster")
        self.declare_parameter("learning_backend_enabled", True)
        self.declare_parameter("primary_topic", "/perception/lidar/pointpillars/cones")
        self.declare_parameter("fallback_topic", "/perception/lidar/cluster/cones")
        self.declare_parameter("output_topic", "/cone_detection_custom")
        self.declare_parameter("primary_metrics_topic", "/perception/lidar/pointpillars/evaluation/metrics")
        self.declare_parameter("fallback_metrics_topic", "/perception/lidar/cluster/evaluation/metrics")
        self.declare_parameter("output_metrics_topic", "/perception/lidar/evaluation/metrics")
        self.declare_parameter("system_health_topic", "/racingbrain/health/system")
        self.declare_parameter("state_topic", "/racingbrain/perception/failure_state")
        self.declare_parameter("decision_period_sec", 0.5)
        self.declare_parameter("backend_stale_timeout_sec", 3.0)
        self.declare_parameter("min_samples", 5)
        self.declare_parameter("history_size", 20)
        self.declare_parameter("empty_ratio_threshold", 0.80)
        self.declare_parameter("latency_warn_ms", 220.0)
        self.declare_parameter("fusion_consistency_min", 0.45)
        self.declare_parameter("fusion_drift_warn", 0.70)
        self.declare_parameter("fallback_hold_sec", 3.0)

        self.mode = str(self.get_parameter("mode").value).strip().lower()
        self.preferred_backend = str(self.get_parameter("preferred_backend").value)
        self.fallback_backend = str(self.get_parameter("fallback_backend").value)
        self.learning_backend_enabled = bool(self.get_parameter("learning_backend_enabled").value)
        self.backend_stale_timeout_sec = max(0.5, float(self.get_parameter("backend_stale_timeout_sec").value))
        self.min_samples = max(1, int(self.get_parameter("min_samples").value))
        history_size = max(self.min_samples, int(self.get_parameter("history_size").value))
        self.empty_ratio_threshold = float(self.get_parameter("empty_ratio_threshold").value)
        self.latency_warn_ms = float(self.get_parameter("latency_warn_ms").value)
        self.fusion_consistency_min = float(self.get_parameter("fusion_consistency_min").value)
        self.fusion_drift_warn = float(self.get_parameter("fusion_drift_warn").value)
        self.fallback_hold_sec = max(0.0, float(self.get_parameter("fallback_hold_sec").value))

        self.primary = BackendWindow(history_size)
        self.fallback = BackendWindow(history_size)
        self.last_system_health: Dict[str, Any] = {}
        self.active_backend = self.preferred_backend if self.learning_backend_enabled else self.fallback_backend
        self.last_switch_wall = time.monotonic()
        self.last_state: Dict[str, Any] = {}

        primary_topic = str(self.get_parameter("primary_topic").value)
        fallback_topic = str(self.get_parameter("fallback_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        primary_metrics_topic = str(self.get_parameter("primary_metrics_topic").value)
        fallback_metrics_topic = str(self.get_parameter("fallback_metrics_topic").value)
        output_metrics_topic = str(self.get_parameter("output_metrics_topic").value)
        system_health_topic = str(self.get_parameter("system_health_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        decision_period_sec = max(0.1, float(self.get_parameter("decision_period_sec").value))

        self.output_pub = self.create_publisher(ThreeDConeArray, output_topic, 10)
        self.metrics_pub = self.create_publisher(String, output_metrics_topic, 10)
        self.state_pub = self.create_publisher(String, state_topic, 10)

        self.create_subscription(ThreeDConeArray, primary_topic, self.cb_primary, 10)
        self.create_subscription(ThreeDConeArray, fallback_topic, self.cb_fallback, 10)
        self.create_subscription(String, primary_metrics_topic, self.cb_primary_metrics, 10)
        self.create_subscription(String, fallback_metrics_topic, self.cb_fallback_metrics, 10)
        self.create_subscription(String, system_health_topic, self.cb_system_health, 10)
        self.create_timer(decision_period_sec, self.on_timer)

        self.get_logger().info(
            "LiDAR backend arbiter ready. "
            f"mode={self.mode}, learning_enabled={self.learning_backend_enabled}, "
            f"preferred={self.preferred_backend}, fallback={self.fallback_backend}"
        )

    def cb_primary(self, msg: ThreeDConeArray) -> None:
        self.primary.push_msg(msg, time.monotonic())
        if self.active_backend == self.preferred_backend:
            self.output_pub.publish(msg)

    def cb_fallback(self, msg: ThreeDConeArray) -> None:
        self.fallback.push_msg(msg, time.monotonic())
        if self.active_backend == self.fallback_backend:
            self.output_pub.publish(msg)

    def cb_primary_metrics(self, msg: String) -> None:
        payload = self.parse_metrics(msg)
        self.primary.push_metrics(payload, time.monotonic())
        if self.active_backend == self.preferred_backend:
            self.publish_selected_metrics(payload)

    def cb_fallback_metrics(self, msg: String) -> None:
        payload = self.parse_metrics(msg)
        self.fallback.push_metrics(payload, time.monotonic())
        if self.active_backend == self.fallback_backend:
            self.publish_selected_metrics(payload)

    def cb_system_health(self, msg: String) -> None:
        try:
            self.last_system_health = json.loads(msg.data)
        except json.JSONDecodeError:
            self.last_system_health = {"raw": msg.data}

    @staticmethod
    def parse_metrics(msg: String) -> Dict[str, Any]:
        try:
            return json.loads(msg.data)
        except json.JSONDecodeError:
            return {"raw": msg.data}

    def publish_selected_metrics(self, payload: Dict[str, Any]) -> None:
        selected = dict(payload)
        selected["selected_by_arbiter"] = True
        selected["selected_backend"] = self.active_backend
        selected["arbiter_mode"] = self.mode
        msg = String()
        msg.data = json.dumps(selected, sort_keys=True)
        self.metrics_pub.publish(msg)

    def health_failure_reasons(self) -> List[str]:
        reasons: List[str] = []
        components = self.last_system_health.get("components", {})
        if not isinstance(components, dict):
            components = {}

        yolo = components.get("yolo", {}) if isinstance(components.get("yolo"), dict) else {}
        fusion = components.get("fusion", {}) if isinstance(components.get("fusion"), dict) else {}
        runtime_budget = (
            self.last_system_health.get("runtime_budget", {})
            if isinstance(self.last_system_health.get("runtime_budget"), dict)
            else {}
        )

        yolo_status = str(yolo.get("status") or "")
        if yolo_status in {"missing", "stale"}:
            reasons.append(f"yolo_{yolo_status}")
        empty_ratio = as_float(yolo.get("empty_ratio"))
        if empty_ratio is not None and empty_ratio > 0.90:
            reasons.append("yolo_empty_ratio_high")

        alignment_state = str(fusion.get("alignment_state") or "")
        consistency = as_float(fusion.get("mean_consistency_score"))
        drift = as_float(fusion.get("mean_calibration_drift_score"))
        stamp_delta_ms = as_float(fusion.get("mean_abs_camera_lidar_stamp_delta_ms"))
        if alignment_state == "time_offset":
            reasons.append("fusion_time_offset_high")
        elif alignment_state == "drift_suspect":
            reasons.append("fusion_calibration_drift_suspect")
        if consistency is not None and consistency < self.fusion_consistency_min:
            reasons.append("fusion_consistency_low")
        if drift is not None and drift > self.fusion_drift_warn:
            reasons.append("fusion_drift_score_high")
        if stamp_delta_ms is not None and stamp_delta_ms > self.backend_stale_timeout_sec * 1000.0:
            reasons.append("fusion_stamp_delta_extreme")

        runtime_budget_state = str(runtime_budget.get("state") or "")
        runtime_budget_score = as_float(runtime_budget.get("score"))
        total_p95_ms = as_float(runtime_budget.get("total_p95_ms"))
        if runtime_budget_state == "freeze":
            reasons.append("runtime_budget_freeze")
        elif runtime_budget_state == "degraded":
            reasons.append("runtime_budget_degraded")
        elif runtime_budget_state == "strained":
            reasons.append("runtime_budget_strained")
        if runtime_budget_score is not None and runtime_budget_score > 0.85:
            reasons.append("runtime_budget_score_high")
        if total_p95_ms is not None and total_p95_ms > 0.0:
            freeze_limit = None
            limits = runtime_budget.get("limits_ms")
            if isinstance(limits, dict):
                freeze_limit = as_float(limits.get("end_to_end_freeze_ms"))
            if freeze_limit is not None and total_p95_ms >= freeze_limit:
                reasons.append("runtime_end_to_end_latency_freeze")
        return reasons

    def backend_failure_reasons(self, wall_time: float) -> List[str]:
        reasons: List[str] = []
        if not self.learning_backend_enabled:
            reasons.append("learning_backend_unavailable")
            return reasons

        if not self.primary.available(wall_time, self.backend_stale_timeout_sec):
            reasons.append("pointpillars_stale")

        if len(self.primary.cone_counts) >= self.min_samples:
            empty_ratio = self.primary.empty_ratio()
            if empty_ratio is not None and empty_ratio > self.empty_ratio_threshold:
                reasons.append("pointpillars_empty_ratio_high")

        mean_total_ms = self.primary.mean_total_ms()
        if mean_total_ms is not None and mean_total_ms > self.latency_warn_ms:
            reasons.append("pointpillars_latency_high")
        return reasons

    def choose_backend(self, wall_time: float, backend_reasons: List[str]) -> str:
        fallback_available = self.fallback.available(wall_time, self.backend_stale_timeout_sec)
        primary_available = self.primary.available(wall_time, self.backend_stale_timeout_sec)
        if self.mode == "cluster":
            return self.fallback_backend
        if self.mode == "pointpillars":
            return self.preferred_backend
        if backend_reasons and fallback_available:
            return self.fallback_backend
        if primary_available and self.learning_backend_enabled:
            if (
                self.active_backend == self.fallback_backend
                and wall_time - self.last_switch_wall < self.fallback_hold_sec
            ):
                return self.fallback_backend
            return self.preferred_backend
        return self.fallback_backend

    def build_task_risk_hint(
        self,
        selected_backend: str,
        health_reasons: List[str],
        backend_reasons: List[str],
        fallback_available: bool,
    ) -> Dict[str, Any]:
        map_contamination_risk = 0.0
        planning_readiness_risk = 0.0
        sources: List[str] = []

        def raise_risk(map_value: float, planning_value: float, source: str) -> None:
            nonlocal map_contamination_risk, planning_readiness_risk
            map_contamination_risk = max(map_contamination_risk, clamp01(map_value))
            planning_readiness_risk = max(planning_readiness_risk, clamp01(planning_value))
            append_unique(sources, source)

        reason_set = sorted(set(health_reasons + backend_reasons))
        for reason in reason_set:
            if reason in {"fusion_time_offset_high", "fusion_consistency_low", "fusion_stamp_delta_extreme"}:
                raise_risk(1.0, 1.0, reason)
            elif reason in {"yolo_empty_ratio_high", "pointpillars_empty_ratio_high"}:
                raise_risk(0.96, 0.92, reason)
            elif reason in {"pointpillars_stale", "yolo_missing", "lidar_missing", "fusion_missing"}:
                raise_risk(0.88, 0.84, reason)
            elif reason in {"runtime_budget_freeze", "runtime_end_to_end_latency_freeze"}:
                raise_risk(0.92, 0.90, reason)
            elif reason in {"runtime_budget_degraded", "runtime_budget_score_high"}:
                raise_risk(0.65, 0.72, reason)
            elif reason in {"runtime_budget_strained"}:
                raise_risk(0.28, 0.36, reason)
            elif reason in {"fusion_calibration_drift_suspect", "fusion_drift_score_high"}:
                raise_risk(0.72, 0.62, reason)
            elif reason in {"yolo_stale"}:
                raise_risk(0.68, 0.58, reason)
            elif reason in {"yolo_latency_high", "lidar_latency_high", "pointpillars_latency_high"}:
                raise_risk(0.50, 0.46, reason)
            elif reason == "learning_backend_unavailable":
                if fallback_available:
                    raise_risk(0.55, 0.45, reason)
                else:
                    raise_risk(0.95, 0.95, reason)
            else:
                raise_risk(0.35, 0.30, reason)

        if (
            self.mode == "auto"
            and self.learning_backend_enabled
            and selected_backend == self.fallback_backend
            and fallback_available
        ):
            raise_risk(0.35, 0.25, "cluster_fallback_active")

        task_risk_hint_score = clamp01(max(map_contamination_risk, planning_readiness_risk))
        if map_contamination_risk >= 0.90:
            state = "freeze"
            world_model_write_policy = "freeze_new_landmarks"
            observation_hit_scale = 0.35
            new_landmarks_allowed = False
        elif task_risk_hint_score >= 0.65:
            state = "degraded"
            world_model_write_policy = "downweight_observations"
            observation_hit_scale = 0.60
            new_landmarks_allowed = True
        elif task_risk_hint_score >= 0.35:
            state = "monitor"
            world_model_write_policy = "monitor_only"
            observation_hit_scale = 0.90
            new_landmarks_allowed = True
        else:
            state = "nominal"
            world_model_write_policy = "open"
            observation_hit_scale = 1.0
            new_landmarks_allowed = True

        return {
            "state": state,
            "task_risk_hint_score": task_risk_hint_score,
            "map_contamination_risk_hint": map_contamination_risk,
            "planning_readiness_risk_hint": planning_readiness_risk,
            "task_risk_hint_sources": sources,
            "task_risk_hint_sources_text": ";".join(sources) if sources else "none",
            "world_model_write_policy_hint": world_model_write_policy,
            "world_model_observation_hit_scale_hint": observation_hit_scale,
            "world_model_new_landmarks_allowed_hint": new_landmarks_allowed,
        }

    def on_timer(self) -> None:
        wall_time = time.monotonic()
        health_reasons = self.health_failure_reasons()
        backend_reasons = self.backend_failure_reasons(wall_time)
        selected = self.choose_backend(wall_time, backend_reasons)
        fallback_available = self.fallback.available(wall_time, self.backend_stale_timeout_sec)
        if selected != self.active_backend:
            self.get_logger().warn(
                f"Switching LiDAR backend: {self.active_backend} -> {selected}; "
                f"backend_reasons={backend_reasons}, health_reasons={health_reasons}"
            )
            self.active_backend = selected
            self.last_switch_wall = wall_time
        task_risk_hint = self.build_task_risk_hint(
            selected_backend=self.active_backend,
            health_reasons=health_reasons,
            backend_reasons=backend_reasons,
            fallback_available=fallback_available,
        )

        payload = {
            "component": "perception_failure_arbiter",
            "stamp": now_sec(self),
            "mode": self.mode,
            "active_lidar_backend": self.active_backend,
            "preferred_backend": self.preferred_backend,
            "fallback_backend": self.fallback_backend,
            "learning_backend_enabled": self.learning_backend_enabled,
            "learning_failed": bool(health_reasons or backend_reasons),
            "backend_failure": bool(backend_reasons),
            "failure_reasons": sorted(set(health_reasons + backend_reasons)),
            "backend_reasons": backend_reasons,
            "health_reasons": health_reasons,
            "primary_available": self.primary.available(wall_time, self.backend_stale_timeout_sec),
            "fallback_available": fallback_available,
            "primary_age_sec": self.primary.age_sec(wall_time),
            "fallback_age_sec": self.fallback.age_sec(wall_time),
            "scores": {
                "primary_empty_ratio": self.primary.empty_ratio(),
                "fallback_empty_ratio": self.fallback.empty_ratio(),
                "primary_mean_cone_count": self.primary.mean_cone_count(),
                "fallback_mean_cone_count": self.fallback.mean_cone_count(),
                "primary_mean_total_ms": self.primary.mean_total_ms(),
                "fallback_mean_total_ms": self.fallback.mean_total_ms(),
            },
            "runtime_budget_state": self.last_system_health.get("runtime_budget_state"),
            "runtime_budget_score": self.last_system_health.get("runtime_budget_score"),
            "runtime_budget_total_p95_ms": self.last_system_health.get("runtime_budget_total_p95_ms"),
            "runtime_budget_sources_text": self.last_system_health.get("runtime_budget_sources_text"),
            "task_risk_hint_state": task_risk_hint["state"],
            "task_risk_hint_score": task_risk_hint["task_risk_hint_score"],
            "map_contamination_risk_hint": task_risk_hint["map_contamination_risk_hint"],
            "planning_readiness_risk_hint": task_risk_hint["planning_readiness_risk_hint"],
            "task_risk_hint_sources": task_risk_hint["task_risk_hint_sources"],
            "task_risk_hint_sources_text": task_risk_hint["task_risk_hint_sources_text"],
            "world_model_write_policy_hint": task_risk_hint["world_model_write_policy_hint"],
            "world_model_observation_hit_scale_hint": task_risk_hint["world_model_observation_hit_scale_hint"],
            "world_model_new_landmarks_allowed_hint": task_risk_hint["world_model_new_landmarks_allowed_hint"],
            "task_risk_hint": task_risk_hint,
        }
        self.last_state = payload
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.state_pub.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = LidarBackendArbiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
