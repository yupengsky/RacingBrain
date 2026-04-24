from __future__ import annotations

import json
import math
import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


STATUS_RANK = {
    "disabled": -1,
    "starting": 0,
    "ok": 1,
    "warn": 2,
    "stale": 3,
    "missing": 4,
}


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def now_sec(node: Node) -> float:
    return float(node.get_clock().now().nanoseconds) * 1e-9


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


def as_bool(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "yes", "on"}:
            return True
        if lowered in {"false", "0", "no", "off"}:
            return False
    return None


def mean(values: List[float]) -> Optional[float]:
    if not values:
        return None
    return sum(values) / float(len(values))


def percentile(values: List[float], fraction: float) -> Optional[float]:
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lo = math.floor(index)
    hi = math.ceil(index)
    if lo == hi:
        return ordered[int(index)]
    weight = index - lo
    return ordered[lo] * (1.0 - weight) + ordered[hi] * weight


def append_unique(values: List[str], item: str) -> None:
    if item and item not in values:
        values.append(item)


class MetricHistory:
    def __init__(self, maxlen: int = 30) -> None:
        self.wall_times: Deque[float] = deque(maxlen=maxlen)
        self.payloads: Deque[Dict[str, Any]] = deque(maxlen=maxlen)

    def push(self, payload: Dict[str, Any], wall_time: float) -> None:
        self.wall_times.append(wall_time)
        self.payloads.append(payload)

    def age_sec(self, wall_time: float) -> Optional[float]:
        if not self.wall_times:
            return None
        return wall_time - self.wall_times[-1]

    def wall_rate_hz(self) -> Optional[float]:
        if len(self.wall_times) < 2:
            return None
        duration = self.wall_times[-1] - self.wall_times[0]
        if duration <= 0.0:
            return None
        return (len(self.wall_times) - 1) / duration

    def latest(self) -> Optional[Dict[str, Any]]:
        if not self.payloads:
            return None
        return self.payloads[-1]

    def numeric_series(self, key: str) -> List[float]:
        values = []
        for payload in self.payloads:
            number = as_float(payload.get(key))
            if number is not None:
                values.append(number)
        return values

    def event_ratio(self, predicate) -> Optional[float]:
        if not self.payloads:
            return None
        hits = 0
        total = 0
        for payload in self.payloads:
            total += 1
            if predicate(payload):
                hits += 1
        if total == 0:
            return None
        return hits / float(total)


class RuntimeHealthMonitor(Node):
    def __init__(self) -> None:
        super().__init__("system_health_monitor")

        self.declare_parameter("expected_perception", True)
        self.declare_parameter("expected_mapping", True)
        self.declare_parameter("selected_lidar_backend", "pointpillars")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("stale_timeout_sec", 3.0)
        self.declare_parameter("startup_grace_sec", 10.0)
        self.declare_parameter("history_size", 30)
        self.declare_parameter("failure_state_topic", "/racingbrain/perception/failure_state")

        self.expected_perception = bool(self.get_parameter("expected_perception").value)
        self.expected_mapping = bool(self.get_parameter("expected_mapping").value)
        self.selected_lidar_backend = str(self.get_parameter("selected_lidar_backend").value)
        self.publish_period_sec = max(0.2, float(self.get_parameter("publish_period_sec").value))
        self.stale_timeout_sec = max(self.publish_period_sec * 1.5, float(self.get_parameter("stale_timeout_sec").value))
        self.startup_grace_sec = max(self.publish_period_sec, float(self.get_parameter("startup_grace_sec").value))
        self.history_size = max(5, int(self.get_parameter("history_size").value))
        self.failure_state_topic = str(self.get_parameter("failure_state_topic").value)

        self.start_wall = time.monotonic()
        self.histories = {
            "yolo": MetricHistory(self.history_size),
            "lidar": MetricHistory(self.history_size),
            "fusion": MetricHistory(self.history_size),
            "mapping": MetricHistory(self.history_size),
        }
        self.last_failure_state: Dict[str, Any] = {}
        self.last_failure_state_wall: Optional[float] = None

        self.health_pub = self.create_publisher(String, "/racingbrain/health/system", 10)
        self.create_subscription(String, "/perception/yolo/evaluation/metrics", self.cb_yolo, 10)
        self.create_subscription(String, "/perception/lidar/evaluation/metrics", self.cb_lidar, 10)
        self.create_subscription(String, "/perception/fusion/evaluation/metrics", self.cb_fusion, 10)
        self.create_subscription(String, "/slam/evaluation/metrics", self.cb_mapping, 10)
        self.create_subscription(String, self.failure_state_topic, self.cb_failure_state, 10)
        self.create_timer(self.publish_period_sec, self.publish_health)

        self.get_logger().info(
            "System health monitor ready. "
            f"expected_perception={self.expected_perception}, "
            f"expected_mapping={self.expected_mapping}, "
            f"lidar_backend={self.selected_lidar_backend}, "
            f"period={self.publish_period_sec:.2f}s"
        )

    def _record(self, component: str, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Failed to parse health metrics for {component}")
            return
        payload.setdefault("component", component)
        self.histories[component].push(payload, time.monotonic())

    def cb_yolo(self, msg: String) -> None:
        self._record("yolo", msg)

    def cb_lidar(self, msg: String) -> None:
        self._record("lidar", msg)

    def cb_fusion(self, msg: String) -> None:
        self._record("fusion", msg)

    def cb_mapping(self, msg: String) -> None:
        self._record("mapping", msg)

    def cb_failure_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"raw": msg.data}
        self.last_failure_state = payload
        self.last_failure_state_wall = time.monotonic()

    def publish_health(self) -> None:
        wall_time = time.monotonic()
        uptime_sec = wall_time - self.start_wall
        components = {
            "yolo": self.build_yolo_status(wall_time, uptime_sec),
            "lidar": self.build_lidar_status(wall_time, uptime_sec),
            "fusion": self.build_fusion_status(wall_time, uptime_sec),
            "mapping": self.build_mapping_status(wall_time, uptime_sec),
        }
        task_risk = self.build_task_risk(components, wall_time, uptime_sec)

        overall_status = "ok"
        alerts: List[str] = []
        relevant_components: List[str] = []
        if self.expected_perception:
            relevant_components.extend(["yolo", "lidar", "fusion"])
        if self.expected_mapping:
            relevant_components.append("mapping")

        best_rank = STATUS_RANK["ok"]
        for name in relevant_components:
            component = components[name]
            best_rank = max(best_rank, STATUS_RANK.get(str(component["status"]), STATUS_RANK["warn"]))
            alerts.extend(component.get("alerts", []))
        alerts.extend(task_risk.get("alerts", []))
        if task_risk["state"] in {"degraded", "freeze"}:
            best_rank = max(best_rank, STATUS_RANK["warn"])

        for status, rank in STATUS_RANK.items():
            if rank == best_rank:
                overall_status = status

        alerts = sorted(set(alerts))
        selected_lidar_backend = str(task_risk.get("selected_lidar_backend") or self.selected_lidar_backend)

        payload = {
            "component": "system_health",
            "stamp": now_sec(self),
            "uptime_sec": uptime_sec,
            "overall_status": overall_status,
            "selected_lidar_backend": selected_lidar_backend,
            "expected_perception": self.expected_perception,
            "expected_mapping": self.expected_mapping,
            "alerts": alerts,
            "components": components,
            "task_risk_state": task_risk["state"],
            "task_risk_score": task_risk["task_risk_score"],
            "map_contamination_risk": task_risk["map_contamination_risk"],
            "planning_readiness_risk": task_risk["planning_readiness_risk"],
            "risk_sources": task_risk["risk_sources"],
            "risk_sources_text": task_risk["risk_sources_text"],
            "world_model_write_policy": task_risk["world_model_write_policy"],
            "world_model_observation_hit_scale": task_risk["observation_hit_scale"],
            "world_model_new_landmarks_allowed": task_risk["new_landmarks_allowed"],
            "task_risk": task_risk,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.health_pub.publish(msg)

    def build_base_status(self, component: str, expected: bool, wall_time: float, uptime_sec: float) -> Dict[str, Any]:
        history = self.histories[component]
        latest = history.latest()
        age_sec = history.age_sec(wall_time)
        status = "ok"
        alerts: List[str] = []
        if not expected:
            status = "disabled"
        elif latest is None:
            status = "starting" if uptime_sec < self.startup_grace_sec else "missing"
            if status == "missing":
                alerts.append(f"{component}_missing")
        elif age_sec is not None and age_sec > self.stale_timeout_sec:
            status = "stale"
            alerts.append(f"{component}_stale")

        total_ms = history.numeric_series("total_ms")
        return {
            "status": status,
            "age_sec": age_sec,
            "sample_count": len(history.payloads),
            "rate_hz": history.wall_rate_hz(),
            "last_event": None if latest is None else latest.get("event"),
            "last_total_ms": None if latest is None else as_float(latest.get("total_ms")),
            "mean_total_ms": mean(total_ms),
            "p95_total_ms": percentile(total_ms, 0.95),
            "alerts": alerts,
        }

    def build_yolo_status(self, wall_time: float, uptime_sec: float) -> Dict[str, Any]:
        info = self.build_base_status("yolo", self.expected_perception, wall_time, uptime_sec)
        history = self.histories["yolo"]
        latest = history.latest()
        cone_counts = history.numeric_series("cone_count")
        empty_ratio = history.event_ratio(
            lambda payload: payload.get("event") == "processed" and as_float(payload.get("cone_count")) == 0.0
        )
        mismatch_rates = []
        for payload in history.payloads:
            checked = as_float(payload.get("hsv_checked_count"))
            mismatch = as_float(payload.get("hsv_mismatch_count"))
            if checked and checked > 0.0 and mismatch is not None:
                mismatch_rates.append(mismatch / checked)

        info.update(
            {
                "mean_cone_count": mean(cone_counts),
                "empty_ratio": empty_ratio,
                "last_cone_count": None if latest is None else as_float(latest.get("cone_count")),
                "hsv_mismatch_ratio": mean(mismatch_rates),
            }
        )

        if info["status"] == "ok":
            if empty_ratio is not None and len(history.payloads) >= 8 and empty_ratio > 0.90:
                info["status"] = "warn"
                info["alerts"].append("yolo_empty_ratio_high")
            elif (info["p95_total_ms"] or 0.0) > 220.0:
                info["status"] = "warn"
                info["alerts"].append("yolo_latency_high")
        return info

    def build_lidar_status(self, wall_time: float, uptime_sec: float) -> Dict[str, Any]:
        info = self.build_base_status("lidar", self.expected_perception, wall_time, uptime_sec)
        history = self.histories["lidar"]
        latest = history.latest()
        cone_key = "cone_count"
        cone_counts = history.numeric_series(cone_key)
        input_age = history.numeric_series("input_age_ms")
        empty_ratio = history.event_ratio(
            lambda payload: payload.get("event") == "processed" and as_float(payload.get(cone_key)) == 0.0
        )

        info.update(
            {
                "backend_component": None if latest is None else latest.get("component"),
                "mean_cone_count": mean(cone_counts),
                "empty_ratio": empty_ratio,
                "mean_input_age_ms": mean(input_age),
                "last_cone_count": None if latest is None else as_float(latest.get(cone_key)),
            }
        )

        if info["status"] == "ok":
            if empty_ratio is not None and len(history.payloads) >= 8 and empty_ratio > 0.95:
                info["status"] = "warn"
                info["alerts"].append("lidar_empty_ratio_high")
            elif (info["p95_total_ms"] or 0.0) > 220.0:
                info["status"] = "warn"
                info["alerts"].append("lidar_latency_high")
        return info

    def build_fusion_status(self, wall_time: float, uptime_sec: float) -> Dict[str, Any]:
        info = self.build_base_status("fusion", self.expected_perception, wall_time, uptime_sec)
        history = self.histories["fusion"]
        latest = history.latest()
        final_counts = history.numeric_series("final_count")
        abs_stamp_deltas = history.numeric_series("abs_camera_lidar_stamp_delta_ms")
        valid_projection_ratios = history.numeric_series("valid_projection_ratio")
        low_iou_ratios = history.numeric_series("low_iou_ratio")
        projection_errors = history.numeric_series("mean_nearest_camera_error_px")
        consistency_scores = history.numeric_series("consistency_score")
        calibration_drift_scores = history.numeric_series("calibration_drift_score")
        magnet_radii = history.numeric_series("magnet_radius_px")
        unknown_ratios = []
        recovered_ratios = []
        force_match_ratios = []
        for payload in history.payloads:
            final_count = as_float(payload.get("final_count"))
            if final_count and final_count > 0.0:
                unknown_count = as_float(payload.get("unknown_count")) or 0.0
                recovered_count = as_float(payload.get("recovered_count")) or 0.0
                force_match_count = as_float(payload.get("force_match_count")) or 0.0
                unknown_ratios.append(unknown_count / final_count)
                recovered_ratios.append(recovered_count / final_count)
                force_match_ratios.append(force_match_count / final_count)

        mean_abs_stamp_delta_ms = mean(abs_stamp_deltas)
        mean_projection_error_px = mean(projection_errors)
        mean_consistency_score = mean(consistency_scores)
        mean_calibration_drift_score = mean(calibration_drift_scores)
        mean_magnet_radius_px = mean(magnet_radii) or 60.0
        alignment_state = "unknown"
        if latest is not None:
            if mean_abs_stamp_delta_ms is not None and mean_abs_stamp_delta_ms > 120.0:
                alignment_state = "time_offset"
            elif (
                mean_calibration_drift_score is not None
                and mean_calibration_drift_score > 0.65
                and mean_projection_error_px is not None
                and mean_projection_error_px > mean_magnet_radius_px
            ):
                alignment_state = "drift_suspect"
            elif mean_consistency_score is not None and mean_consistency_score < 0.45:
                alignment_state = "degraded"
            else:
                alignment_state = "ok"

        info.update(
            {
                "mean_final_count": mean(final_counts),
                "last_final_count": None if latest is None else as_float(latest.get("final_count")),
                "mean_unknown_ratio": mean(unknown_ratios),
                "mean_recovered_ratio": mean(recovered_ratios),
                "mean_force_match_ratio": mean(force_match_ratios),
                "mean_abs_camera_lidar_stamp_delta_ms": mean_abs_stamp_delta_ms,
                "mean_valid_projection_ratio": mean(valid_projection_ratios),
                "mean_low_iou_ratio": mean(low_iou_ratios),
                "mean_projection_error_px": mean_projection_error_px,
                "p95_projection_error_px": percentile(projection_errors, 0.95),
                "mean_consistency_score": mean_consistency_score,
                "last_consistency_score": None if latest is None else as_float(latest.get("consistency_score")),
                "mean_calibration_drift_score": mean_calibration_drift_score,
                "last_calibration_drift_score": None if latest is None else as_float(latest.get("calibration_drift_score")),
                "mean_magnet_radius_px": mean_magnet_radius_px,
                "alignment_state": alignment_state,
            }
        )

        if info["status"] == "ok":
            if alignment_state == "time_offset":
                info["status"] = "warn"
                info["alerts"].append("fusion_time_offset_high")
            elif alignment_state == "drift_suspect":
                info["status"] = "warn"
                info["alerts"].append("fusion_calibration_drift_suspect")
            elif alignment_state == "degraded":
                info["status"] = "warn"
                info["alerts"].append("fusion_consistency_low")
            elif (info["mean_unknown_ratio"] or 0.0) > 0.75:
                info["status"] = "warn"
                info["alerts"].append("fusion_unknown_ratio_high")
            elif (
                info["mean_projection_error_px"] is not None
                and info["mean_projection_error_px"] > mean_magnet_radius_px * 1.5
            ):
                info["status"] = "warn"
                info["alerts"].append("fusion_projection_residual_high")
            elif (info["mean_final_count"] or 0.0) < 1.0 and len(history.payloads) >= 8:
                info["status"] = "warn"
                info["alerts"].append("fusion_output_low")
        return info

    def build_mapping_status(self, wall_time: float, uptime_sec: float) -> Dict[str, Any]:
        info = self.build_base_status("mapping", self.expected_mapping, wall_time, uptime_sec)
        history = self.histories["mapping"]
        latest = history.latest()
        stable_cones = history.numeric_series("stable_cones")
        sync_callback_ms = history.numeric_series("sync_callback_ms")
        gate_hit_scales = history.numeric_series("risk_gate_hit_scale")
        rejected_risk_gate = history.numeric_series("rejected_risk_gate")
        downweighted = history.numeric_series("risk_gate_downweighted_observations")
        utilization = []
        for payload in history.payloads:
            total = as_float(payload.get("observations_total"))
            used = as_float(payload.get("observations_used"))
            if total and total > 0.0 and used is not None:
                utilization.append(used / total)

        info.update(
            {
                "last_stable_cones": None if latest is None else as_float(latest.get("stable_cones")),
                "mean_stable_cones": mean(stable_cones),
                "mean_observation_utilization": mean(utilization),
                "mean_sync_callback_ms": mean(sync_callback_ms),
                "map_locked": None if latest is None else as_bool(latest.get("map_locked")),
                "last_risk_gate_state": None if latest is None else latest.get("risk_gate_state"),
                "last_risk_gate_reasons": None if latest is None else latest.get("risk_gate_reasons"),
                "last_risk_gate_new_cones_allowed": None if latest is None else as_bool(latest.get("risk_gate_new_cones_allowed")),
                "mean_risk_gate_hit_scale": mean(gate_hit_scales),
                "rejected_risk_gate_total": sum(rejected_risk_gate),
                "risk_gate_downweighted_total": sum(downweighted),
            }
        )

        if info["status"] == "ok":
            if info["last_risk_gate_state"] in {"degraded", "freeze"}:
                info["status"] = "warn"
                info["alerts"].append("mapping_risk_gate_active")
            elif uptime_sec > max(self.startup_grace_sec * 2.0, 15.0) and (info["last_stable_cones"] or 0.0) < 1.0:
                info["status"] = "warn"
                info["alerts"].append("mapping_no_stable_cones")
            elif (info["mean_sync_callback_ms"] or 0.0) > 120.0:
                info["status"] = "warn"
                info["alerts"].append("mapping_sync_latency_high")
        return info

    def build_task_risk(
        self,
        components: Dict[str, Dict[str, Any]],
        wall_time: float,
        uptime_sec: float,
    ) -> Dict[str, Any]:
        map_contamination_risk = 0.0
        planning_readiness_risk = 0.0
        risk_sources: List[str] = []

        def raise_risk(map_value: Optional[float], planning_value: Optional[float], source: str) -> None:
            nonlocal map_contamination_risk, planning_readiness_risk
            if map_value is not None:
                map_contamination_risk = max(map_contamination_risk, clamp01(map_value))
            if planning_value is not None:
                planning_readiness_risk = max(planning_readiness_risk, clamp01(planning_value))
            append_unique(risk_sources, source)

        failure_state = {}
        failure_state_fresh = False
        if self.last_failure_state_wall is not None:
            failure_state_age = wall_time - self.last_failure_state_wall
            failure_state_fresh = failure_state_age <= self.stale_timeout_sec
            if failure_state_fresh:
                failure_state = self.last_failure_state

        selected_lidar_backend = str(failure_state.get("active_lidar_backend") or self.selected_lidar_backend)

        hint_map = as_float(failure_state.get("map_contamination_risk_hint"))
        hint_planning = as_float(failure_state.get("planning_readiness_risk_hint"))
        hint_score = as_float(failure_state.get("task_risk_hint_score"))
        if hint_map is not None:
            map_contamination_risk = max(map_contamination_risk, clamp01(hint_map))
        if hint_planning is not None:
            planning_readiness_risk = max(planning_readiness_risk, clamp01(hint_planning))
        elif hint_score is not None:
            planning_readiness_risk = max(planning_readiness_risk, clamp01(hint_score))
        if hint_score is not None:
            map_contamination_risk = max(map_contamination_risk, clamp01(hint_score))

        hint_sources_text = str(failure_state.get("task_risk_hint_sources_text") or "")
        for item in hint_sources_text.split(";"):
            append_unique(risk_sources, item.strip())

        if bool(failure_state.get("learning_failed")):
            raise_risk(0.55, 0.50, "learning_failure_active")
        if bool(failure_state.get("backend_failure")):
            raise_risk(0.78, 0.72, "lidar_backend_failure")
        if (
            failure_state_fresh
            and self.learning_fallback_active(failure_state)
            and str(failure_state.get("mode") or "auto") == "auto"
        ):
            raise_risk(0.35, 0.25, "cluster_fallback_active")

        yolo = components.get("yolo", {})
        lidar = components.get("lidar", {})
        fusion = components.get("fusion", {})
        mapping = components.get("mapping", {})

        if str(yolo.get("status")) in {"missing", "stale"}:
            raise_risk(0.82, 0.76, f"yolo_{yolo.get('status')}")
        if str(lidar.get("status")) in {"missing", "stale"}:
            raise_risk(0.84, 0.80, f"lidar_{lidar.get('status')}")
        if str(fusion.get("status")) in {"missing", "stale"}:
            raise_risk(0.96, 0.94, f"fusion_{fusion.get('status')}")
        if str(mapping.get("status")) in {"missing", "stale"}:
            raise_risk(0.90, 0.92, f"mapping_{mapping.get('status')}")

        if self.enough_samples(yolo, 8):
            empty_ratio = as_float(yolo.get("empty_ratio"))
            if empty_ratio is not None and empty_ratio > 0.90:
                raise_risk(0.96, 0.92, "yolo_empty_ratio_high")
        if (as_float(yolo.get("p95_total_ms")) or 0.0) > 220.0:
            raise_risk(0.50, 0.46, "yolo_latency_high")

        if self.enough_samples(lidar, 8):
            empty_ratio = as_float(lidar.get("empty_ratio"))
            if empty_ratio is not None and empty_ratio > 0.95:
                raise_risk(0.88, 0.82, "lidar_empty_ratio_high")
        if (as_float(lidar.get("p95_total_ms")) or 0.0) > 220.0:
            raise_risk(0.52, 0.48, "lidar_latency_high")

        alignment_state = str(fusion.get("alignment_state") or "")
        if alignment_state == "time_offset":
            raise_risk(1.0, 1.0, "fusion_time_offset_high")
        elif alignment_state == "drift_suspect":
            raise_risk(0.72, 0.62, "fusion_calibration_drift_suspect")
        elif alignment_state == "degraded":
            raise_risk(0.85, 0.76, "fusion_consistency_low")

        unknown_ratio = as_float(fusion.get("mean_unknown_ratio"))
        if unknown_ratio is not None and unknown_ratio > 0.75:
            raise_risk(0.60, 0.52, "fusion_unknown_ratio_high")

        projection_error_px = as_float(fusion.get("mean_projection_error_px"))
        magnet_radius_px = as_float(fusion.get("mean_magnet_radius_px")) or 60.0
        if projection_error_px is not None and projection_error_px > magnet_radius_px * 1.5:
            raise_risk(0.66, 0.58, "fusion_projection_residual_high")

        gate_state = str(mapping.get("last_risk_gate_state") or "")
        if gate_state == "freeze":
            raise_risk(1.0, 0.96, "mapping_freeze_active")
        elif gate_state == "degraded":
            raise_risk(0.78, 0.70, "mapping_gate_active")

        observation_utilization = as_float(mapping.get("mean_observation_utilization"))
        if self.enough_samples(mapping, 8) and observation_utilization is not None and observation_utilization < 0.45:
            raise_risk(0.58, 0.54, "mapping_observation_utilization_low")

        last_stable_cones = as_float(mapping.get("last_stable_cones"))
        if uptime_sec > max(self.startup_grace_sec * 2.0, 15.0):
            if last_stable_cones is None or last_stable_cones < 6.0:
                raise_risk(0.40, 0.82, "stable_map_not_ready")
            elif last_stable_cones < 12.0:
                raise_risk(0.22, 0.45, "stable_map_thin")

        task_risk_score = clamp01(max(map_contamination_risk, planning_readiness_risk))
        if map_contamination_risk >= 0.90:
            state = "freeze"
            world_model_write_policy = "freeze_new_landmarks"
            observation_hit_scale = 0.35
            new_landmarks_allowed = False
        elif task_risk_score >= 0.65:
            state = "degraded"
            world_model_write_policy = "downweight_observations"
            observation_hit_scale = 0.60
            new_landmarks_allowed = True
        elif task_risk_score >= 0.35:
            state = "monitor"
            world_model_write_policy = "monitor_only"
            observation_hit_scale = 0.90
            new_landmarks_allowed = True
        else:
            state = "nominal"
            world_model_write_policy = "open"
            observation_hit_scale = 1.0
            new_landmarks_allowed = True

        alerts: List[str] = []
        if state == "freeze":
            alerts.extend(["task_risk_freeze", "map_contamination_risk_high"])
        elif state == "degraded":
            alerts.extend(["task_risk_degraded", "map_contamination_risk_elevated"])
        elif state == "monitor":
            alerts.append("task_risk_monitor")
        if planning_readiness_risk >= 0.80:
            alerts.append("planning_readiness_risk_high")

        return {
            "state": state,
            "task_risk_score": task_risk_score,
            "map_contamination_risk": map_contamination_risk,
            "planning_readiness_risk": planning_readiness_risk,
            "risk_sources": risk_sources,
            "risk_sources_text": ";".join(risk_sources) if risk_sources else "none",
            "world_model_write_policy": world_model_write_policy,
            "observation_hit_scale": observation_hit_scale,
            "new_landmarks_allowed": new_landmarks_allowed,
            "selected_lidar_backend": selected_lidar_backend,
            "alerts": sorted(set(alerts)),
        }

    @staticmethod
    def enough_samples(component: Dict[str, Any], minimum: int) -> bool:
        sample_count = component.get("sample_count")
        try:
            return int(sample_count) >= minimum
        except (TypeError, ValueError):
            return False

    @staticmethod
    def learning_fallback_active(failure_state: Dict[str, Any]) -> bool:
        if not failure_state:
            return False
        if not bool(failure_state.get("fallback_available")):
            return False
        return str(failure_state.get("active_lidar_backend") or "") == "cluster"


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = RuntimeHealthMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
