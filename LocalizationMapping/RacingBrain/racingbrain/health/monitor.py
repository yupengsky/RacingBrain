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

        self.expected_perception = bool(self.get_parameter("expected_perception").value)
        self.expected_mapping = bool(self.get_parameter("expected_mapping").value)
        self.selected_lidar_backend = str(self.get_parameter("selected_lidar_backend").value)
        self.publish_period_sec = max(0.2, float(self.get_parameter("publish_period_sec").value))
        self.stale_timeout_sec = max(self.publish_period_sec * 1.5, float(self.get_parameter("stale_timeout_sec").value))
        self.startup_grace_sec = max(self.publish_period_sec, float(self.get_parameter("startup_grace_sec").value))
        self.history_size = max(5, int(self.get_parameter("history_size").value))

        self.start_wall = time.monotonic()
        self.histories = {
            "yolo": MetricHistory(self.history_size),
            "lidar": MetricHistory(self.history_size),
            "fusion": MetricHistory(self.history_size),
            "mapping": MetricHistory(self.history_size),
        }

        self.health_pub = self.create_publisher(String, "/racingbrain/health/system", 10)
        self.create_subscription(String, "/perception/yolo/evaluation/metrics", self.cb_yolo, 10)
        self.create_subscription(String, "/perception/lidar/evaluation/metrics", self.cb_lidar, 10)
        self.create_subscription(String, "/perception/fusion/evaluation/metrics", self.cb_fusion, 10)
        self.create_subscription(String, "/slam/evaluation/metrics", self.cb_mapping, 10)
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

    def publish_health(self) -> None:
        wall_time = time.monotonic()
        uptime_sec = wall_time - self.start_wall
        components = {
            "yolo": self.build_yolo_status(wall_time, uptime_sec),
            "lidar": self.build_lidar_status(wall_time, uptime_sec),
            "fusion": self.build_fusion_status(wall_time, uptime_sec),
            "mapping": self.build_mapping_status(wall_time, uptime_sec),
        }

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

        for status, rank in STATUS_RANK.items():
            if rank == best_rank:
                overall_status = status

        payload = {
            "component": "system_health",
            "stamp": now_sec(self),
            "uptime_sec": uptime_sec,
            "overall_status": overall_status,
            "selected_lidar_backend": self.selected_lidar_backend,
            "expected_perception": self.expected_perception,
            "expected_mapping": self.expected_mapping,
            "alerts": alerts,
            "components": components,
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
            }
        )

        if info["status"] == "ok":
            if uptime_sec > max(self.startup_grace_sec * 2.0, 15.0) and (info["last_stable_cones"] or 0.0) < 1.0:
                info["status"] = "warn"
                info["alerts"].append("mapping_no_stable_cones")
            elif (info["mean_sync_callback_ms"] or 0.0) > 120.0:
                info["status"] = "warn"
                info["alerts"].append("mapping_sync_latency_high")
        return info


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
