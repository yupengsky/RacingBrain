#!/usr/bin/env python3
"""Sidecar evaluator for the RacingBrain perception-to-mapping chain.

The monitor subscribes to runtime ROS topics and writes self-consistency metrics
without changing the online control path. It intentionally avoids ground-truth
claims; when no annotated map or reference trajectory is available, the reported
metrics are health, latency, consistency, and duplicate-risk indicators.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from cone_interfaces.msg import ConeArray
from drd25_msgs.msg import Map
from gnss_ins_msg.msg import Gnssins64
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from test_cone_segmentation.msg import ThreeDConeArray
from visualization_msgs.msg import Marker, MarkerArray


COLOR_NAMES = {
    0: "blue",
    1: "red",
    2: "yellow_big",
    3: "yellow_small",
    4: "unknown",
}


def stamp_to_float(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def msg_stamp(msg: Any) -> Optional[float]:
    header = getattr(msg, "header", None)
    if header is None:
        return None
    return stamp_to_float(header.stamp)


def percentile(sorted_values: List[float], p: float) -> Optional[float]:
    if not sorted_values:
        return None
    if len(sorted_values) == 1:
        return sorted_values[0]
    idx = (len(sorted_values) - 1) * p
    lo = math.floor(idx)
    hi = math.ceil(idx)
    if lo == hi:
        return sorted_values[int(idx)]
    frac = idx - lo
    return sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac


def summarize(values: Iterable[float]) -> Dict[str, Optional[float]]:
    data = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    if not data:
        return {
            "count": 0,
            "min": None,
            "max": None,
            "mean": None,
            "median": None,
            "p95": None,
            "stddev": None,
        }
    ordered = sorted(data)
    return {
        "count": len(ordered),
        "min": ordered[0],
        "max": ordered[-1],
        "mean": statistics.fmean(ordered),
        "median": percentile(ordered, 0.5),
        "p95": percentile(ordered, 0.95),
        "stddev": statistics.pstdev(ordered) if len(ordered) > 1 else 0.0,
    }


def nearest_distances(points: List[Tuple[float, float]]) -> List[float]:
    if len(points) < 2:
        return []
    distances: List[float] = []
    for i, (x0, y0) in enumerate(points):
        best = None
        for j, (x1, y1) in enumerate(points):
            if i == j:
                continue
            dist = math.hypot(x0 - x1, y0 - y1)
            if best is None or dist < best:
                best = dist
        if best is not None:
            distances.append(best)
    return distances


def duplicate_pair_count(points: List[Tuple[float, float]], threshold: float) -> int:
    count = 0
    for i, (x0, y0) in enumerate(points):
        for x1, y1 in points[i + 1 :]:
            if math.hypot(x0 - x1, y0 - y1) < threshold:
                count += 1
    return count


def point_bounds(points: List[Tuple[float, float]]) -> Dict[str, Optional[float]]:
    if not points:
        return {
            "min_x": None,
            "max_x": None,
            "min_y": None,
            "max_y": None,
            "span_x": None,
            "span_y": None,
        }
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return {
        "min_x": min(xs),
        "max_x": max(xs),
        "min_y": min(ys),
        "max_y": max(ys),
        "span_x": max(xs) - min(xs),
        "span_y": max(ys) - min(ys),
    }


def summarize_processing_times(rows: Iterable[Dict[str, Any]]) -> Dict[str, Dict[str, Dict[str, Optional[float]]]]:
    grouped: Dict[str, Dict[str, List[float]]] = defaultdict(lambda: defaultdict(list))
    for row in rows:
        component = str(row.get("component") or "unknown")
        for key, value in row.items():
            if not key.endswith("_ms"):
                continue
            try:
                number = float(value)
            except (TypeError, ValueError):
                continue
            if math.isfinite(number):
                grouped[component][key].append(number)
    return {
        component: {stage: summarize(values) for stage, values in sorted(stages.items())}
        for component, stages in sorted(grouped.items())
    }


def summarize_component_metrics(rows: Iterable[Dict[str, Any]], component: str, keys: Iterable[str]) -> Dict[str, Dict[str, Optional[float]]]:
    grouped: Dict[str, List[float]] = {key: [] for key in keys}
    for row in rows:
        if str(row.get("component") or "") != component:
            continue
        for key in keys:
            try:
                value = float(row.get(key))
            except (TypeError, ValueError):
                continue
            if math.isfinite(value):
                grouped[key].append(value)
    return {key: summarize(values) for key, values in grouped.items()}


class TopicStats:
    def __init__(self) -> None:
        self.count = 0
        self.first_wall: Optional[float] = None
        self.last_wall: Optional[float] = None
        self.first_stamp: Optional[float] = None
        self.last_stamp: Optional[float] = None
        self.wall_gaps: List[float] = []
        self.stamp_gaps: List[float] = []

    def mark(self, wall_time: float, stamp: Optional[float]) -> None:
        self.count += 1
        if self.last_wall is not None:
            self.wall_gaps.append(wall_time - self.last_wall)
        if stamp is not None and self.last_stamp is not None:
            self.stamp_gaps.append(stamp - self.last_stamp)

        if self.first_wall is None:
            self.first_wall = wall_time
        self.last_wall = wall_time

        if stamp is not None:
            if self.first_stamp is None:
                self.first_stamp = stamp
            self.last_stamp = stamp

    def summary(self, start_wall: float) -> Dict[str, Any]:
        wall_duration = None
        stamp_duration = None
        wall_rate = None
        stamp_rate = None
        if self.first_wall is not None and self.last_wall is not None:
            wall_duration = max(0.0, self.last_wall - self.first_wall)
            if wall_duration > 0.0:
                wall_rate = self.count / wall_duration
        if self.first_stamp is not None and self.last_stamp is not None:
            stamp_duration = max(0.0, self.last_stamp - self.first_stamp)
            if stamp_duration > 0.0:
                stamp_rate = self.count / stamp_duration

        return {
            "count": self.count,
            "first_wall_sec": None if self.first_wall is None else self.first_wall - start_wall,
            "last_wall_sec": None if self.last_wall is None else self.last_wall - start_wall,
            "wall_duration_sec": wall_duration,
            "wall_rate_hz": wall_rate,
            "first_stamp": self.first_stamp,
            "last_stamp": self.last_stamp,
            "stamp_duration_sec": stamp_duration,
            "stamp_rate_hz": stamp_rate,
            "wall_gap_sec": summarize(self.wall_gaps),
            "stamp_gap_sec": summarize(self.stamp_gaps),
        }


class EvalMonitor(Node):
    def __init__(self, duplicate_threshold: float) -> None:
        super().__init__("drd26_eval_monitor")
        self.start_wall = time.time()
        self.last_msg_wall: Optional[float] = None
        self.duplicate_threshold = duplicate_threshold

        self.topic_stats: Dict[str, TopicStats] = defaultdict(TopicStats)
        self.last_stamp_by_name: Dict[str, float] = {}
        self.topic_rows: List[Dict[str, Any]] = []
        self.latency_rows: List[Dict[str, Any]] = []
        self.fusion_rows: List[Dict[str, Any]] = []
        self.map_rows: List[Dict[str, Any]] = []
        self.odom_rows: List[Dict[str, Any]] = []
        self.mapping_debug_rows: List[Dict[str, Any]] = []
        self.health_rows: List[Dict[str, Any]] = []
        self.timing_rows: List[Dict[str, Any]] = []

        self.yolo_counts: List[int] = []
        self.lidar_cone_counts: List[int] = []
        self.fusion_counts: List[int] = []
        self.fusion_unknown_ratios: List[float] = []
        self.global_counts: List[int] = []
        self.gnss_speeds: List[float] = []
        self.odom_speeds: List[float] = []
        self.final_map_points: List[Tuple[float, float]] = []
        self.final_map_color_counts: Counter[str] = Counter()
        self.last_system_health: Optional[Dict[str, Any]] = None
        self.last_non_stale_system_health: Optional[Dict[str, Any]] = None

        self.odom_start: Optional[Tuple[float, float]] = None
        self.odom_last: Optional[Tuple[float, float]] = None
        self.odom_length = 0.0
        self.odom_final: Optional[Tuple[float, float]] = None

        self.create_subscription(Image, "/camera1/image_raw", self.cb_image, 10)
        self.create_subscription(PointCloud2, "/lidar_points", self.cb_lidar_points, 10)
        self.create_subscription(Gnssins64, "/gongji_gnss_ins_64", self.cb_gnss, 10)
        self.create_subscription(ConeArray, "/yolo/cones", self.cb_yolo, 10)
        self.create_subscription(ThreeDConeArray, "/cone_detection_custom", self.cb_lidar_cones, 10)
        self.create_subscription(Map, "/perception/fusion/map", self.cb_fusion, 10)
        self.create_subscription(MarkerArray, "/global_map", self.cb_global_map, 10)
        self.create_subscription(Odometry, "/vehicle_odom", self.cb_odom, 10)
        self.create_subscription(NavPath, "/vehicle_path", self.cb_path, 10)
        self.create_subscription(String, "/perception/yolo/evaluation/metrics", self.cb_yolo_metrics, 10)
        self.create_subscription(String, "/perception/lidar/evaluation/metrics", self.cb_lidar_metrics, 10)
        self.create_subscription(String, "/perception/fusion/evaluation/metrics", self.cb_fusion_metrics, 10)
        self.create_subscription(String, "/slam/evaluation/metrics", self.cb_mapping_debug, 10)
        self.create_subscription(String, "/racingbrain/health/system", self.cb_system_health, 10)

    def mark(self, topic: str, stamp: Optional[float], extra: Optional[Dict[str, Any]] = None) -> None:
        now = time.time()
        self.last_msg_wall = now
        self.topic_stats[topic].mark(now, stamp)
        if stamp is not None:
            self.last_stamp_by_name[topic] = stamp
        row = {
            "elapsed_wall_sec": now - self.start_wall,
            "topic": topic,
            "stamp": stamp,
        }
        if extra:
            row.update(extra)
        self.topic_rows.append(row)

    def record_latency(self, name: str, target_stamp: Optional[float], ref_stamp: Optional[float]) -> None:
        if target_stamp is None or ref_stamp is None:
            return
        delta = target_stamp - ref_stamp
        if not math.isfinite(delta):
            return
        self.latency_rows.append(
            {
                "elapsed_wall_sec": time.time() - self.start_wall,
                "name": name,
                "delta_sec": delta,
                "abs_delta_sec": abs(delta),
            }
        )

    def cb_image(self, msg: Image) -> None:
        stamp = msg_stamp(msg)
        self.mark(
            "/camera1/image_raw",
            stamp,
            {"width": msg.width, "height": msg.height, "frame_id": msg.header.frame_id},
        )

    def cb_lidar_points(self, msg: PointCloud2) -> None:
        stamp = msg_stamp(msg)
        point_count = int(msg.width) * int(msg.height)
        self.mark(
            "/lidar_points",
            stamp,
            {"point_count": point_count, "frame_id": msg.header.frame_id},
        )

    def cb_gnss(self, msg: Gnssins64) -> None:
        stamp = msg_stamp(msg)
        speed = math.hypot(float(msg.vel_e), float(msg.vel_n))
        self.gnss_speeds.append(speed)
        self.mark(
            "/gongji_gnss_ins_64",
            stamp,
            {
                "speed": speed,
                "yaw_deg": float(msg.yaw),
                "gyro_z_deg_s": float(msg.imu_gyro_z),
            },
        )

    def cb_yolo(self, msg: ConeArray) -> None:
        stamp = msg_stamp(msg)
        count = len(msg.cones)
        self.yolo_counts.append(count)
        color_counts = Counter(COLOR_NAMES.get(int(c.color), str(int(c.color))) for c in msg.cones)
        confidences = [float(c.confidence) for c in msg.cones]
        self.mark(
            "/yolo/cones",
            stamp,
            {
                "cone_count": count,
                "mean_confidence": summarize(confidences)["mean"],
                "color_counts": json.dumps(dict(color_counts), sort_keys=True),
            },
        )
        self.record_latency("yolo_stamp_minus_last_image_stamp", stamp, self.last_stamp_by_name.get("/camera1/image_raw"))

    def cb_lidar_cones(self, msg: ThreeDConeArray) -> None:
        stamp = msg_stamp(msg)
        count = len(msg.cones)
        self.lidar_cone_counts.append(count)
        points = [(float(c.center.x), float(c.center.y)) for c in msg.cones]
        distances = [math.hypot(x, y) for x, y in points]
        self.mark(
            "/cone_detection_custom",
            stamp,
            {
                "cone_count": count,
                "mean_distance": summarize(distances)["mean"],
                "max_distance": summarize(distances)["max"],
            },
        )
        self.record_latency("lidar_cones_stamp_minus_last_pointcloud_stamp", stamp, self.last_stamp_by_name.get("/lidar_points"))

    def cb_fusion(self, msg: Map) -> None:
        stamp = msg_stamp(msg)
        points = [(float(c.x), float(c.y)) for c in msg.track]
        colors = [int(c.color) for c in msg.track]
        color_counts = Counter(COLOR_NAMES.get(c, str(c)) for c in colors)
        unknown_count = color_counts.get("unknown", 0)
        count = len(points)
        unknown_ratio = float(unknown_count) / float(count) if count else 0.0
        distances = [math.hypot(x, y) for x, y in points]
        nnd = nearest_distances(points)
        duplicate_pairs = duplicate_pair_count(points, self.duplicate_threshold)

        self.fusion_counts.append(count)
        self.fusion_unknown_ratios.append(unknown_ratio)
        row = {
            "elapsed_wall_sec": time.time() - self.start_wall,
            "stamp": stamp,
            "cone_count": count,
            "unknown_count": unknown_count,
            "unknown_ratio": unknown_ratio,
            "mean_distance": summarize(distances)["mean"],
            "max_distance": summarize(distances)["max"],
            "nearest_neighbor_min": summarize(nnd)["min"],
            "duplicate_pairs": duplicate_pairs,
            "color_counts": json.dumps(dict(color_counts), sort_keys=True),
        }
        self.fusion_rows.append(row)
        self.mark("/perception/fusion/map", stamp, row)

        self.record_latency("fusion_stamp_minus_last_yolo_stamp", stamp, self.last_stamp_by_name.get("/yolo/cones"))
        self.record_latency("fusion_stamp_minus_last_lidar_cones_stamp", stamp, self.last_stamp_by_name.get("/cone_detection_custom"))
        self.record_latency("fusion_stamp_minus_last_gnss_stamp", stamp, self.last_stamp_by_name.get("/gongji_gnss_ins_64"))

    def cb_global_map(self, msg: MarkerArray) -> None:
        stamp = None
        points: List[Tuple[float, float]] = []
        color_counts: Counter[str] = Counter()
        for marker in msg.markers:
            if marker.action == Marker.DELETEALL:
                continue
            if stamp is None:
                stamp = stamp_to_float(marker.header.stamp)
            points.append((float(marker.pose.position.x), float(marker.pose.position.y)))
            color_counts[self.marker_color_name(marker)] += 1
        if stamp is None and msg.markers:
            stamp = stamp_to_float(msg.markers[0].header.stamp)

        count = len(points)
        nnd = nearest_distances(points)
        bounds = point_bounds(points)
        duplicate_pairs = duplicate_pair_count(points, self.duplicate_threshold)
        self.global_counts.append(count)
        self.final_map_points = points
        self.final_map_color_counts = color_counts

        row = {
            "elapsed_wall_sec": time.time() - self.start_wall,
            "stamp": stamp,
            "stable_cone_count": count,
            "nearest_neighbor_min": summarize(nnd)["min"],
            "nearest_neighbor_mean": summarize(nnd)["mean"],
            "duplicate_pairs": duplicate_pairs,
            "color_counts": json.dumps(dict(color_counts), sort_keys=True),
            **bounds,
        }
        self.map_rows.append(row)
        self.mark("/global_map", stamp, row)
        self.record_latency("global_map_stamp_minus_last_fusion_stamp", stamp, self.last_stamp_by_name.get("/perception/fusion/map"))
        self.record_latency("global_map_stamp_minus_last_gnss_stamp", stamp, self.last_stamp_by_name.get("/gongji_gnss_ins_64"))

    def cb_odom(self, msg: Odometry) -> None:
        stamp = msg_stamp(msg)
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        speed = math.hypot(float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y))
        if self.odom_start is None:
            self.odom_start = (x, y)
        if self.odom_last is not None:
            self.odom_length += math.hypot(x - self.odom_last[0], y - self.odom_last[1])
        self.odom_last = (x, y)
        self.odom_final = (x, y)
        self.odom_speeds.append(speed)
        row = {
            "elapsed_wall_sec": time.time() - self.start_wall,
            "stamp": stamp,
            "x": x,
            "y": y,
            "speed": speed,
            "trajectory_length": self.odom_length,
        }
        self.odom_rows.append(row)
        self.mark("/vehicle_odom", stamp, row)

    def cb_path(self, msg: NavPath) -> None:
        stamp = msg_stamp(msg)
        self.mark("/vehicle_path", stamp, {"pose_count": len(msg.poses)})

    def record_timing_metrics(self, msg: String, topic: str, default_component: str) -> Dict[str, Any]:
        now = time.time()
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {"raw": msg.data}
        data.setdefault("component", default_component)
        data["elapsed_wall_sec"] = now - self.start_wall
        self.timing_rows.append(data)
        self.mark(topic, data.get("stamp"), data)
        return data

    def cb_yolo_metrics(self, msg: String) -> None:
        self.record_timing_metrics(msg, "/perception/yolo/evaluation/metrics", "yolo")

    def cb_lidar_metrics(self, msg: String) -> None:
        self.record_timing_metrics(msg, "/perception/lidar/evaluation/metrics", "lidar_cluster")

    def cb_fusion_metrics(self, msg: String) -> None:
        self.record_timing_metrics(msg, "/perception/fusion/evaluation/metrics", "fusion")

    def cb_mapping_debug(self, msg: String) -> None:
        data = self.record_timing_metrics(msg, "/slam/evaluation/metrics", "mapping")
        self.mapping_debug_rows.append(data)

    def cb_system_health(self, msg: String) -> None:
        now = time.time()
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {"raw": msg.data}

        stamp_raw = data.get("stamp")
        try:
            stamp = float(stamp_raw) if stamp_raw is not None else None
        except (TypeError, ValueError):
            stamp = None

        components = data.get("components", {}) if isinstance(data.get("components"), dict) else {}
        row = {
            "elapsed_wall_sec": now - self.start_wall,
            "stamp": stamp,
            "overall_status": data.get("overall_status"),
            "selected_lidar_backend": data.get("selected_lidar_backend"),
            "alert_count": len(data.get("alerts", [])) if isinstance(data.get("alerts"), list) else 0,
            "alerts": json.dumps(data.get("alerts", []), ensure_ascii=False),
        }
        for component_name in ("yolo", "lidar", "fusion", "mapping"):
            component = components.get(component_name, {})
            row[f"{component_name}_status"] = component.get("status")
            row[f"{component_name}_rate_hz"] = component.get("rate_hz")
        if isinstance(components.get("fusion"), dict):
            row["fusion_unknown_ratio"] = components["fusion"].get("mean_unknown_ratio")
            row["fusion_force_match_ratio"] = components["fusion"].get("mean_force_match_ratio")
            row["fusion_alignment_state"] = components["fusion"].get("alignment_state")
            row["fusion_consistency_score"] = components["fusion"].get("mean_consistency_score")
            row["fusion_calibration_drift_score"] = components["fusion"].get("mean_calibration_drift_score")
            row["fusion_projection_error_px"] = components["fusion"].get("mean_projection_error_px")
            row["fusion_stamp_delta_ms"] = components["fusion"].get("mean_abs_camera_lidar_stamp_delta_ms")
            row["fusion_low_iou_ratio"] = components["fusion"].get("mean_low_iou_ratio")
            row["fusion_valid_projection_ratio"] = components["fusion"].get("mean_valid_projection_ratio")
        if isinstance(components.get("mapping"), dict):
            row["mapping_stable_cones"] = components["mapping"].get("last_stable_cones")
            row["mapping_observation_utilization"] = components["mapping"].get("mean_observation_utilization")
        if isinstance(components.get("lidar"), dict):
            row["lidar_backend_component"] = components["lidar"].get("backend_component")

        self.health_rows.append(row)
        self.last_system_health = data
        if str(data.get("overall_status")) not in {"stale", "missing"}:
            self.last_non_stale_system_health = data
        self.mark("/racingbrain/health/system", stamp, row)

    @staticmethod
    def marker_color_name(marker: Marker) -> str:
        r = float(marker.color.r)
        g = float(marker.color.g)
        b = float(marker.color.b)
        if b > 0.7 and r < 0.3 and g < 0.3:
            return "blue"
        if r > 0.7 and g < 0.3 and b < 0.3:
            return "red"
        if r > 0.7 and g > 0.7 and b < 0.3:
            return "yellow"
        return "unknown"

    def build_summary(self, dataset: Optional[str], bag_metadata: Dict[str, Any]) -> Dict[str, Any]:
        topic_summaries = {
            topic: stats.summary(self.start_wall)
            for topic, stats in sorted(self.topic_stats.items())
        }
        latency_by_name: Dict[str, List[float]] = defaultdict(list)
        for row in self.latency_rows:
            latency_by_name[str(row["name"])].append(float(row["delta_sec"]))

        final_duplicate_pairs = duplicate_pair_count(self.final_map_points, self.duplicate_threshold)
        final_nnd = nearest_distances(self.final_map_points)
        start_end_distance = None
        if self.odom_start is not None and self.odom_final is not None:
            start_end_distance = math.hypot(
                self.odom_final[0] - self.odom_start[0],
                self.odom_final[1] - self.odom_start[1],
            )

        return {
            "success": (
                topic_summaries.get("/perception/fusion/map", {}).get("count", 0) > 0
                and topic_summaries.get("/global_map", {}).get("count", 0) > 0
                and topic_summaries.get("/racingbrain/health/system", {}).get("count", 0) > 0
            ),
            "dataset": dataset,
            "bag_metadata": bag_metadata,
            "elapsed_wall_sec": time.time() - self.start_wall,
            "duplicate_threshold_m": self.duplicate_threshold,
            "topics": topic_summaries,
            "latency_sec": {
                name: summarize(values)
                for name, values in sorted(latency_by_name.items())
            },
            "processing_time_ms": summarize_processing_times(self.timing_rows),
            "fusion_consistency": summarize_component_metrics(
                self.timing_rows,
                "fusion",
                (
                    "abs_camera_lidar_stamp_delta_ms",
                    "valid_projection_ratio",
                    "iou_match_ratio",
                    "unmatched_camera_ratio",
                    "low_iou_ratio",
                    "mean_nearest_camera_error_px",
                    "p95_nearest_camera_error_px",
                    "mean_best_iou",
                    "unknown_ratio",
                    "force_match_ratio",
                    "consistency_score",
                    "calibration_drift_score",
                ),
            ),
            "perception": {
                "yolo_cones_per_frame": summarize(self.yolo_counts),
                "lidar_cones_per_frame": summarize(self.lidar_cone_counts),
                "fused_cones_per_frame": summarize(self.fusion_counts),
                "fused_unknown_ratio": summarize(self.fusion_unknown_ratios),
            },
            "map": {
                "stable_cones_per_frame": summarize(self.global_counts),
                "final_stable_cones": len(self.final_map_points),
                "final_duplicate_pairs": final_duplicate_pairs,
                "final_nearest_neighbor_m": summarize(final_nnd),
                "final_bounds": point_bounds(self.final_map_points),
                "final_color_counts": dict(self.final_map_color_counts),
            },
            "trajectory": {
                "odom_length_m": self.odom_length,
                "start_end_distance_m": start_end_distance,
                "gnss_speed_mps": summarize(self.gnss_speeds),
                "odom_speed_mps": summarize(self.odom_speeds),
            },
            "mapping_debug": {
                "enabled": bool(self.mapping_debug_rows),
                "frames": len(self.mapping_debug_rows),
                "created_cones_total": sum(int(r.get("created_cones", 0)) for r in self.mapping_debug_rows),
                "matched_cones_total": sum(int(r.get("matched_cones", 0)) for r in self.mapping_debug_rows),
                "removed_cones_total": sum(int(r.get("removed_cones", 0)) for r in self.mapping_debug_rows),
                "missed_in_view_total": sum(int(r.get("missed_in_view", 0)) for r in self.mapping_debug_rows),
                "unknown_observations_total": sum(int(r.get("observations_unknown", 0)) for r in self.mapping_debug_rows),
                "last": self.mapping_debug_rows[-1] if self.mapping_debug_rows else None,
            },
            "system_health": {
                "frames": len(self.health_rows),
                "overall_status_counts": dict(Counter(str(r.get("overall_status")) for r in self.health_rows if r.get("overall_status") is not None)),
                "last": self.last_system_health,
                "last_non_stale": self.last_non_stale_system_health,
            },
        }


def write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames: List[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def read_bag_metadata(dataset: Optional[str]) -> Dict[str, Any]:
    if not dataset:
        return {}
    metadata_path = Path(dataset) / "metadata.yaml"
    if not metadata_path.exists():
        return {}
    try:
        import yaml  # type: ignore
    except Exception:
        return {"metadata_path": str(metadata_path)}
    try:
        data = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    except Exception as exc:
        return {"metadata_path": str(metadata_path), "error": str(exc)}
    info = data.get("rosbag2_bagfile_information", {})
    topics = {}
    for item in info.get("topics_with_message_count", []):
        metadata = item.get("topic_metadata", {})
        name = metadata.get("name")
        if not name:
            continue
        topics[name] = {
            "type": metadata.get("type"),
            "message_count": item.get("message_count"),
        }
    return {
        "metadata_path": str(metadata_path),
        "duration_sec": info.get("duration", {}).get("nanoseconds", 0) / 1e9,
        "message_count": info.get("message_count"),
        "topics": topics,
    }


def write_report(log_dir: Path, summary: Dict[str, Any]) -> None:
    topics = summary.get("topics", {})
    scenario = summary.get("scenario", {}) or {}
    perception = summary.get("perception", {})
    map_metrics = summary.get("map", {})
    trajectory = summary.get("trajectory", {})
    mapping_debug = summary.get("mapping_debug", {})
    system_health = summary.get("system_health", {})
    processing_time = summary.get("processing_time_ms", {})
    fusion_consistency = summary.get("fusion_consistency", {})

    def fmt(value: Any, digits: int = 3) -> str:
        if value is None:
            return "n/a"
        if isinstance(value, float):
            return f"{value:.{digits}f}"
        return str(value)

    lines = [
        "# RacingBrain Mapping Evaluation Report",
        "",
        f"- Success: `{summary.get('success')}`",
        f"- Dataset: `{summary.get('dataset')}`",
        f"- Scenario: `{scenario.get('profile', 'none')}`",
        f"- Elapsed wall time: `{fmt(summary.get('elapsed_wall_sec'))} s`",
        f"- Duplicate threshold: `{fmt(summary.get('duplicate_threshold_m'))} m`",
        "",
    ]
    if scenario:
        lines.extend(
            [
                "## Scenario",
                "",
                f"- Profile: `{scenario.get('profile')}`",
                f"- Fault start: `{fmt(scenario.get('fault_start_sec'))} s`",
                f"- Fault duration: `{fmt(scenario.get('fault_duration_sec'))} s`",
                f"- Fault injector enabled: `{scenario.get('use_fault_injector')}`",
                f"- Calibration override: `{scenario.get('fusion_calibration_file')}`",
                "",
            ]
        )
    lines.extend(
        [
        "## Topic Health",
        "",
        "| Topic | Count | Wall Hz | Stamp Hz | First Wall s |",
        "|---|---:|---:|---:|---:|",
        ]
    )
    for topic, stats in topics.items():
        lines.append(
            "| {topic} | {count} | {wall_hz} | {stamp_hz} | {first} |".format(
                topic=topic,
                count=stats.get("count", 0),
                wall_hz=fmt(stats.get("wall_rate_hz")),
                stamp_hz=fmt(stats.get("stamp_rate_hz")),
                first=fmt(stats.get("first_wall_sec")),
            )
        )

    lines.extend(
        [
            "",
            "## Perception",
            "",
            f"- YOLO cones/frame mean: `{fmt(perception.get('yolo_cones_per_frame', {}).get('mean'))}`",
            f"- LiDAR cones/frame mean: `{fmt(perception.get('lidar_cones_per_frame', {}).get('mean'))}`",
            f"- Fused cones/frame mean: `{fmt(perception.get('fused_cones_per_frame', {}).get('mean'))}`",
            f"- Fused UNKNOWN ratio mean: `{fmt(perception.get('fused_unknown_ratio', {}).get('mean'))}`",
            f"- Fusion consistency score mean: `{fmt(fusion_consistency.get('consistency_score', {}).get('mean'))}`",
            f"- Fusion calibration drift score mean: `{fmt(fusion_consistency.get('calibration_drift_score', {}).get('mean'))}`",
            f"- Fusion projection error mean: `{fmt(fusion_consistency.get('mean_nearest_camera_error_px', {}).get('mean'))} px`",
            f"- Fusion camera-LiDAR stamp delta mean: `{fmt(fusion_consistency.get('abs_camera_lidar_stamp_delta_ms', {}).get('mean'))} ms`",
            "",
            "## System Health",
            "",
            f"- Health frames: `{system_health.get('frames')}`",
            f"- Health status counts: `{json.dumps(system_health.get('overall_status_counts', {}), ensure_ascii=False)}`",
            f"- Last overall status: `{(system_health.get('last') or {}).get('overall_status')}`",
            f"- Last non-stale status: `{(system_health.get('last_non_stale') or {}).get('overall_status')}`",
            "",
            "## Processing Time",
            "",
            "| Component | Stage | Frames | Mean ms | P95 ms |",
            "|---|---|---:|---:|---:|",
        ]
    )
    for component, stages in processing_time.items():
        for stage, stats in stages.items():
            lines.append(
                "| {component} | {stage} | {count} | {mean} | {p95} |".format(
                    component=component,
                    stage=stage,
                    count=stats.get("count", 0),
                    mean=fmt(stats.get("mean")),
                    p95=fmt(stats.get("p95")),
                )
            )

    lines.extend(
        [
            "",
            "## Map",
            "",
            f"- Final stable cones: `{map_metrics.get('final_stable_cones')}`",
            f"- Final duplicate pairs: `{map_metrics.get('final_duplicate_pairs')}`",
            f"- Final nearest-neighbor mean: `{fmt(map_metrics.get('final_nearest_neighbor_m', {}).get('mean'))} m`",
            f"- Final color counts: `{json.dumps(map_metrics.get('final_color_counts', {}), ensure_ascii=False)}`",
            "",
            "## Trajectory",
            "",
            f"- Odom path length: `{fmt(trajectory.get('odom_length_m'))} m`",
            f"- Start-end distance: `{fmt(trajectory.get('start_end_distance_m'))} m`",
            f"- GNSS speed mean: `{fmt(trajectory.get('gnss_speed_mps', {}).get('mean'))} m/s`",
            "",
            "## Mapping Debug",
            "",
            f"- Debug topic enabled: `{mapping_debug.get('enabled')}`",
            f"- Debug frames: `{mapping_debug.get('frames')}`",
            f"- Created cones total: `{mapping_debug.get('created_cones_total')}`",
            f"- Matched cones total: `{mapping_debug.get('matched_cones_total')}`",
            f"- Removed cones total: `{mapping_debug.get('removed_cones_total')}`",
            f"- Unknown observations total: `{mapping_debug.get('unknown_observations_total')}`",
            "",
            "## Notes",
            "",
            "These are self-consistency metrics, not absolute accuracy metrics. "
            "Absolute map and trajectory errors require annotated cone positions or a reference trajectory.",
        ]
    )
    (log_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_topic_rates(log_dir: Path, summary: Dict[str, Any]) -> None:
    rows = []
    for topic, stats in summary.get("topics", {}).items():
        rows.append(
            {
                "topic": topic,
                "count": stats.get("count"),
                "wall_rate_hz": stats.get("wall_rate_hz"),
                "stamp_rate_hz": stats.get("stamp_rate_hz"),
                "first_wall_sec": stats.get("first_wall_sec"),
                "last_wall_sec": stats.get("last_wall_sec"),
                "stamp_gap_mean_sec": stats.get("stamp_gap_sec", {}).get("mean"),
                "stamp_gap_p95_sec": stats.get("stamp_gap_sec", {}).get("p95"),
            }
        )
    write_csv(log_dir / "topic_rates.csv", rows)


def maybe_write_plots(log_dir: Path, monitor: EvalMonitor) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return

    plot_dir = log_dir / "plots"
    plot_dir.mkdir(exist_ok=True)

    if monitor.fusion_rows or monitor.map_rows:
        plt.figure(figsize=(9, 4))
        if monitor.fusion_rows:
            plt.plot(
                [r["elapsed_wall_sec"] for r in monitor.fusion_rows],
                [r["cone_count"] for r in monitor.fusion_rows],
                label="fused cones",
            )
        if monitor.map_rows:
            plt.plot(
                [r["elapsed_wall_sec"] for r in monitor.map_rows],
                [r["stable_cone_count"] for r in monitor.map_rows],
                label="stable global cones",
            )
        plt.xlabel("wall time (s)")
        plt.ylabel("count")
        plt.legend()
        plt.tight_layout()
        plt.savefig(plot_dir / "cone_counts.png")
        plt.close()

    if monitor.latency_rows:
        by_name: Dict[str, List[float]] = defaultdict(list)
        for row in monitor.latency_rows:
            by_name[str(row["name"])].append(float(row["delta_sec"]) * 1000.0)
        plt.figure(figsize=(10, 5))
        labels = []
        data = []
        for name, values in sorted(by_name.items()):
            if values:
                labels.append(name.replace("_stamp_minus_", "\nminus\n"))
                data.append(values)
        if data:
            plt.boxplot(data, labels=labels, showfliers=False)
            plt.ylabel("stamp delta (ms)")
            plt.xticks(rotation=20, ha="right")
            plt.tight_layout()
            plt.savefig(plot_dir / "latency_boxplot.png")
        plt.close()

    if monitor.timing_rows:
        by_component: Dict[str, List[float]] = defaultdict(list)
        for row in monitor.timing_rows:
            try:
                total_ms = float(row.get("total_ms"))
            except (TypeError, ValueError):
                continue
            if math.isfinite(total_ms):
                by_component[str(row.get("component") or "unknown")].append(total_ms)
        if by_component:
            plt.figure(figsize=(9, 5))
            labels = []
            data = []
            for component, values in sorted(by_component.items()):
                if values:
                    labels.append(component)
                    data.append(values)
            if data:
                plt.boxplot(data, labels=labels, showfliers=False)
                plt.ylabel("total processing time (ms)")
                plt.xticks(rotation=20, ha="right")
                plt.tight_layout()
                plt.savefig(plot_dir / "processing_time_boxplot.png")
            plt.close()

    if monitor.odom_rows:
        plt.figure(figsize=(6, 6))
        plt.plot([r["x"] for r in monitor.odom_rows], [r["y"] for r in monitor.odom_rows], label="odom")
        if monitor.final_map_points:
            plt.scatter(
                [p[0] for p in monitor.final_map_points],
                [p[1] for p in monitor.final_map_points],
                s=20,
                label="final stable cones",
            )
        plt.axis("equal")
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.legend()
        plt.tight_layout()
        plt.savefig(plot_dir / "trajectory_and_map.png")
        plt.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", required=True, help="Directory for evaluation artifacts")
    parser.add_argument("--dataset", default=None, help="Rosbag dataset directory")
    parser.add_argument("--scenario-file", default=None, help="Optional JSON file describing the fault scenario")
    parser.add_argument("--timeout", type=float, default=90.0, help="Maximum monitor wall time")
    parser.add_argument("--idle-timeout", type=float, default=8.0, help="Stop after no messages for this many seconds")
    parser.add_argument("--duplicate-threshold", type=float, default=0.75, help="Distance threshold for duplicate cone risk")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    bag_metadata = read_bag_metadata(args.dataset)
    scenario = None
    if args.scenario_file:
        scenario_path = Path(args.scenario_file)
        if scenario_path.exists():
            scenario = json.loads(scenario_path.read_text(encoding="utf-8"))

    rclpy.init()
    monitor = EvalMonitor(duplicate_threshold=args.duplicate_threshold)
    deadline = time.time() + float(args.timeout)
    try:
        while time.time() < deadline:
            rclpy.spin_once(monitor, timeout_sec=0.1)
            if (
                args.idle_timeout > 0.0
                and monitor.last_msg_wall is not None
                and time.time() - monitor.last_msg_wall > args.idle_timeout
            ):
                break
    finally:
        summary = monitor.build_summary(args.dataset, bag_metadata)
        summary["scenario"] = scenario
        (log_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        write_topic_rates(log_dir, summary)
        write_csv(log_dir / "topic_samples.csv", monitor.topic_rows)
        write_csv(log_dir / "latency.csv", monitor.latency_rows)
        write_csv(log_dir / "fusion_frames.csv", monitor.fusion_rows)
        write_csv(log_dir / "map_frames.csv", monitor.map_rows)
        write_csv(log_dir / "odom.csv", monitor.odom_rows)
        write_csv(log_dir / "mapping_debug_frames.csv", monitor.mapping_debug_rows)
        write_csv(log_dir / "system_health.csv", monitor.health_rows)
        write_csv(log_dir / "processing_times.csv", monitor.timing_rows)
        write_report(log_dir, summary)
        maybe_write_plots(log_dir, monitor)
        print(json.dumps(summary, indent=2, ensure_ascii=False))
        monitor.destroy_node()
        rclpy.shutdown()

    return 0 if summary.get("success") else 2


if __name__ == "__main__":
    raise SystemExit(main())
