from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

from cone_interfaces.msg import ConeArray
from test_cone_segmentation.msg import ThreeDCone, ThreeDConeArray


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def stamp_sec(msg: Any) -> float:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return 0.0
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def now_sec(node: Node) -> float:
    return float(node.get_clock().now().nanoseconds) * 1e-9


@dataclass
class Calibration:
    image_width: int
    image_height: int
    fx: float
    fy: float
    cx: float
    cy: float
    t_lidar_to_camera: np.ndarray


@dataclass
class TemporalTrack:
    center: np.ndarray
    last_seen_wall: float
    hits: int = 1


@dataclass
class VerificationResult:
    accepted: bool
    geometry_score: float
    camera_score: float
    temporal_score: float
    total_score: float
    local_point_count: int
    height: float
    xy_span: float
    center_error: float
    reason: str
    matched_track: Optional[TemporalTrack]
    output_center: np.ndarray


class PointPillarsLocalVerifier(Node):
    """Low-cost gate for PointPillars boxes using local point-cloud evidence."""

    def __init__(self) -> None:
        super().__init__("pointpillars_local_verifier")

        self.declare_parameter("input_cloud_topic", "/lidar_points")
        self.declare_parameter("input_cones_topic", "/perception/lidar/pointpillars/raw_cones")
        self.declare_parameter("camera_cones_topic", "/yolo/cones")
        self.declare_parameter("output_topic", "/perception/lidar/pointpillars/cones")
        self.declare_parameter("metrics_topic", "/perception/lidar/pointpillars/verifier/metrics")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("max_cloud_age_sec", 0.30)
        self.declare_parameter("camera_stale_timeout_sec", 0.45)
        self.declare_parameter("max_points_for_verify", 80000)
        self.declare_parameter("min_points", 3)
        self.declare_parameter("xy_margin", 0.25)
        self.declare_parameter("min_box_half_extent", 0.25)
        self.declare_parameter("max_box_half_extent", 0.85)
        self.declare_parameter("z_margin", 0.35)
        self.declare_parameter("min_height", 0.04)
        self.declare_parameter("max_height", 0.90)
        self.declare_parameter("max_xy_span", 1.10)
        self.declare_parameter("center_tolerance", 0.55)
        self.declare_parameter("geometry_pass_threshold", 0.55)
        self.declare_parameter("total_pass_threshold", 0.58)
        self.declare_parameter("temporal_match_distance", 0.70)
        self.declare_parameter("temporal_track_ttl_sec", 0.70)
        self.declare_parameter("temporal_rescue_min_hits", 2)
        self.declare_parameter("temporal_smoothing_alpha", 0.85)
        self.declare_parameter("passthrough_on_missing_cloud", False)

        self.max_cloud_age_sec = float(self.get_parameter("max_cloud_age_sec").value)
        self.camera_stale_timeout_sec = float(self.get_parameter("camera_stale_timeout_sec").value)
        self.max_points_for_verify = int(self.get_parameter("max_points_for_verify").value)
        self.min_points = int(self.get_parameter("min_points").value)
        self.xy_margin = float(self.get_parameter("xy_margin").value)
        self.min_box_half_extent = float(self.get_parameter("min_box_half_extent").value)
        self.max_box_half_extent = float(self.get_parameter("max_box_half_extent").value)
        self.z_margin = float(self.get_parameter("z_margin").value)
        self.min_height = float(self.get_parameter("min_height").value)
        self.max_height = float(self.get_parameter("max_height").value)
        self.max_xy_span = float(self.get_parameter("max_xy_span").value)
        self.center_tolerance = float(self.get_parameter("center_tolerance").value)
        self.geometry_pass_threshold = float(self.get_parameter("geometry_pass_threshold").value)
        self.total_pass_threshold = float(self.get_parameter("total_pass_threshold").value)
        self.temporal_match_distance = float(self.get_parameter("temporal_match_distance").value)
        self.temporal_track_ttl_sec = float(self.get_parameter("temporal_track_ttl_sec").value)
        self.temporal_rescue_min_hits = int(self.get_parameter("temporal_rescue_min_hits").value)
        self.temporal_smoothing_alpha = clamp01(float(self.get_parameter("temporal_smoothing_alpha").value))
        self.passthrough_on_missing_cloud = bool(self.get_parameter("passthrough_on_missing_cloud").value)

        self.last_cloud_msg: Optional[PointCloud2] = None
        self.last_camera_msg: Optional[ConeArray] = None
        self.last_camera_wall: Optional[float] = None
        self.tracks: Deque[TemporalTrack] = deque(maxlen=80)
        self.calibration = self.load_calibration(str(self.get_parameter("calibration_file").value))

        input_cloud_topic = str(self.get_parameter("input_cloud_topic").value)
        input_cones_topic = str(self.get_parameter("input_cones_topic").value)
        camera_cones_topic = str(self.get_parameter("camera_cones_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        metrics_topic = str(self.get_parameter("metrics_topic").value)

        self.output_pub = self.create_publisher(ThreeDConeArray, output_topic, 10)
        self.metrics_pub = self.create_publisher(String, metrics_topic, 10)
        self.create_subscription(PointCloud2, input_cloud_topic, self.cloud_callback, qos_profile_sensor_data)
        self.create_subscription(ThreeDConeArray, input_cones_topic, self.cones_callback, 10)
        self.create_subscription(ConeArray, camera_cones_topic, self.camera_callback, 10)

        self.get_logger().info(
            "PointPillars local verifier ready. "
            f"cloud={input_cloud_topic}, input={input_cones_topic}, output={output_topic}, "
            f"camera_gate={'on' if self.calibration else 'off'}"
        )

    def load_calibration(self, calibration_file: str) -> Optional[Calibration]:
        candidates: List[Path] = []
        if calibration_file:
            candidates.append(Path(calibration_file))
        try:
            candidates.append(Path(get_package_share_directory("fs_fusion_box")) / "config" / "calibration.yaml")
        except PackageNotFoundError:
            pass

        path = next((item for item in candidates if item.exists()), None)
        if path is None:
            if candidates:
                self.get_logger().warn(
                    "Verifier calibration file not found: "
                    + ", ".join(str(item) for item in candidates)
                )
            return None
        try:
            import yaml

            data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            params = data.get("/fusion_box_node", {}).get("ros__parameters", {})
            camera_matrix = params.get("camera_matrix", {})
            matrix = np.asarray(params.get("lidar_to_camera_matrix", []), dtype=np.float64).reshape(4, 4)
            return Calibration(
                image_width=int(params.get("image_width", 640)),
                image_height=int(params.get("image_height", 480)),
                fx=float(camera_matrix.get("fx")),
                fy=float(camera_matrix.get("fy")),
                cx=float(camera_matrix.get("cx")),
                cy=float(camera_matrix.get("cy")),
                t_lidar_to_camera=matrix,
            )
        except Exception as exc:
            self.get_logger().warn(f"Failed to load verifier calibration: {exc}")
            return None

    def cloud_callback(self, msg: PointCloud2) -> None:
        self.last_cloud_msg = msg

    def camera_callback(self, msg: ConeArray) -> None:
        self.last_camera_msg = msg
        self.last_camera_wall = time.monotonic()

    def cones_callback(self, msg: ThreeDConeArray) -> None:
        start = time.perf_counter()
        wall_time = time.monotonic()

        cloud_msg = self.last_cloud_msg
        if cloud_msg is None or abs(stamp_sec(msg) - stamp_sec(cloud_msg)) > self.max_cloud_age_sec:
            if self.passthrough_on_missing_cloud:
                self.output_pub.publish(msg)
                self.publish_metrics(
                    msg,
                    start,
                    event="passthrough_missing_cloud",
                    input_count=len(msg.cones),
                    accepted_count=len(msg.cones),
                    rejected_count=0,
                    details=[],
                )
            else:
                empty = ThreeDConeArray()
                empty.header = msg.header
                self.output_pub.publish(empty)
                self.publish_metrics(
                    msg,
                    start,
                    event="rejected_missing_cloud",
                    input_count=len(msg.cones),
                    accepted_count=0,
                    rejected_count=len(msg.cones),
                    details=[],
                )
            return

        points = self.pointcloud_to_xyz(cloud_msg)
        if points.size == 0:
            empty = ThreeDConeArray()
            empty.header = msg.header
            self.output_pub.publish(empty)
            self.publish_metrics(
                msg,
                start,
                event="rejected_empty_cloud",
                input_count=len(msg.cones),
                accepted_count=0,
                rejected_count=len(msg.cones),
                details=[],
            )
            return

        self.prune_tracks(wall_time)
        output = ThreeDConeArray()
        output.header = msg.header
        details: List[Dict[str, Any]] = []

        for cone in msg.cones:
            result = self.verify_cone(cone, points, wall_time)
            details.append(
                {
                    "accepted": result.accepted,
                    "reason": result.reason,
                    "geometry_score": result.geometry_score,
                    "camera_score": result.camera_score,
                    "temporal_score": result.temporal_score,
                    "total_score": result.total_score,
                    "local_point_count": result.local_point_count,
                    "height": result.height,
                    "xy_span": result.xy_span,
                    "center_error": result.center_error,
                }
            )
            if result.accepted:
                verified = ThreeDCone()
                verified.center.x = float(result.output_center[0])
                verified.center.y = float(result.output_center[1])
                verified.center.z = float(result.output_center[2])
                verified.size = cone.size
                output.cones.append(verified)
                self.update_track(result.output_center, result.matched_track, wall_time)

        self.output_pub.publish(output)
        self.publish_metrics(
            msg,
            start,
            event="processed",
            input_count=len(msg.cones),
            accepted_count=len(output.cones),
            rejected_count=len(msg.cones) - len(output.cones),
            details=details,
        )

    def pointcloud_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        try:
            points = self.pointcloud_to_xyz_from_buffer(msg)
        except Exception:
            raw = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            arr = np.asarray(list(raw))
            if arr.dtype.names:
                points = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float32, copy=False)
            else:
                points = np.asarray(arr, dtype=np.float32).reshape(-1, 3)

        if self.max_points_for_verify > 0 and points.shape[0] > self.max_points_for_verify:
            stride = int(math.ceil(points.shape[0] / float(self.max_points_for_verify)))
            points = points[::stride]
        return points

    @staticmethod
    def pointcloud_to_xyz_from_buffer(msg: PointCloud2) -> np.ndarray:
        offsets: Dict[str, int] = {}
        for field in msg.fields:
            if field.name in {"x", "y", "z"}:
                offsets[field.name] = int(field.offset)
        if set(offsets) != {"x", "y", "z"}:
            raise ValueError("PointCloud2 is missing one of x/y/z fields.")

        endian = ">" if msg.is_bigendian else "<"
        dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": [endian + "f4", endian + "f4", endian + "f4"],
                "offsets": [offsets["x"], offsets["y"], offsets["z"]],
                "itemsize": int(msg.point_step),
            }
        )
        point_count = int(msg.width) * int(msg.height)
        packed_row_step = int(msg.point_step) * int(msg.width)
        if int(msg.height) <= 1 or int(msg.row_step) == packed_row_step:
            arr = np.frombuffer(msg.data, dtype=dtype, count=point_count)
        else:
            rows = []
            for row in range(int(msg.height)):
                start = row * int(msg.row_step)
                end = start + packed_row_step
                rows.append(np.frombuffer(msg.data[start:end], dtype=dtype, count=int(msg.width)))
            arr = np.concatenate(rows) if rows else np.asarray([], dtype=dtype)
        points = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float32, copy=False)
        finite = np.isfinite(points).all(axis=1)
        return points[finite]

    def verify_cone(self, cone: ThreeDCone, points: np.ndarray, wall_time: float) -> VerificationResult:
        center = np.asarray([cone.center.x, cone.center.y, cone.center.z], dtype=np.float32)
        local = self.crop_local_points(cone, points)
        geometry_score, local_count, height, xy_span, center_error, geometry_reason = self.geometry_score(center, local)
        camera_score = self.camera_score(center)
        matched_track, temporal_score = self.temporal_score(center, wall_time)

        total_score = clamp01(0.65 * geometry_score + 0.20 * camera_score + 0.15 * temporal_score)
        geometry_pass = geometry_score >= self.geometry_pass_threshold and local_count >= self.min_points
        temporal_rescue = (
            geometry_score >= self.geometry_pass_threshold * 0.75
            and matched_track is not None
            and matched_track.hits >= self.temporal_rescue_min_hits
        )
        accepted = (geometry_pass and total_score >= self.total_pass_threshold) or temporal_rescue

        reason = geometry_reason
        if accepted and temporal_rescue and not geometry_pass:
            reason = "temporal_rescue"
        elif accepted:
            reason = "verified"

        output_center = center
        if accepted and matched_track is not None:
            alpha = self.temporal_smoothing_alpha
            output_center = alpha * center + (1.0 - alpha) * matched_track.center

        return VerificationResult(
            accepted=accepted,
            geometry_score=geometry_score,
            camera_score=camera_score,
            temporal_score=temporal_score,
            total_score=total_score,
            local_point_count=local_count,
            height=height,
            xy_span=xy_span,
            center_error=center_error,
            reason=reason,
            matched_track=matched_track,
            output_center=output_center,
        )

    def crop_local_points(self, cone: ThreeDCone, points: np.ndarray) -> np.ndarray:
        cx, cy, cz = float(cone.center.x), float(cone.center.y), float(cone.center.z)
        half_x = self.clamped_half_extent(float(cone.size.x))
        half_y = self.clamped_half_extent(float(cone.size.y))
        half_z = max(abs(float(cone.size.z)) * 0.5 + self.z_margin, self.max_height)

        mask = (
            (np.abs(points[:, 0] - cx) <= half_x)
            & (np.abs(points[:, 1] - cy) <= half_y)
            & (np.abs(points[:, 2] - cz) <= half_z)
        )
        return points[mask]

    def clamped_half_extent(self, size_value: float) -> float:
        half = abs(size_value) * 0.5 + self.xy_margin
        return max(self.min_box_half_extent, min(self.max_box_half_extent, half))

    def geometry_score(self, center: np.ndarray, local: np.ndarray) -> Tuple[float, int, float, float, float, str]:
        local_count = int(local.shape[0])
        if local_count == 0:
            return 0.0, 0, 0.0, 0.0, float("inf"), "no_local_points"

        mins = np.min(local, axis=0)
        maxs = np.max(local, axis=0)
        span = maxs - mins
        height = float(max(0.0, span[2]))
        xy_span = float(max(span[0], span[1]))
        centroid = np.mean(local[:, :2], axis=0)
        center_error = float(np.linalg.norm(centroid - center[:2]))

        point_score = clamp01(local_count / max(1.0, float(self.min_points * 2)))
        height_score = 1.0 if self.min_height <= height <= self.max_height else 0.35
        width_score = 1.0 if xy_span <= self.max_xy_span else max(0.0, 1.0 - (xy_span - self.max_xy_span))
        center_score = max(0.0, 1.0 - center_error / max(1e-3, self.center_tolerance))

        score = clamp01(
            0.35 * point_score
            + 0.20 * height_score
            + 0.20 * width_score
            + 0.25 * center_score
        )

        if local_count < self.min_points:
            reason = "too_few_points"
        elif xy_span > self.max_xy_span:
            reason = "wide_local_cluster"
        elif center_error > self.center_tolerance:
            reason = "center_offset_high"
        elif height < self.min_height:
            reason = "height_too_low"
        elif height > self.max_height:
            reason = "height_too_high"
        else:
            reason = "geometry_ok"
        return score, local_count, height, xy_span, center_error, reason

    def camera_score(self, center_lidar: np.ndarray) -> float:
        if self.calibration is None or self.last_camera_msg is None or self.last_camera_wall is None:
            return 0.5
        if time.monotonic() - self.last_camera_wall > self.camera_stale_timeout_sec:
            return 0.5

        uv = self.project_lidar_point(center_lidar)
        if uv is None:
            return 0.5

        u, v = uv
        best = 0.0
        for cone in self.last_camera_msg.cones:
            cx = float(cone.center.x)
            cy = float(cone.center.y)
            half_w = max(1.0, float(cone.size.x) * 0.5)
            half_h = max(1.0, float(cone.size.y) * 0.5)
            dx = abs(u - cx)
            dy = abs(v - cy)
            inside = dx <= half_w and dy <= half_h
            if inside:
                best = max(best, float(getattr(cone, "confidence", 1.0)))
                continue
            norm = math.sqrt((dx / half_w) ** 2 + (dy / half_h) ** 2)
            best = max(best, max(0.0, 1.0 - 0.5 * norm) * float(getattr(cone, "confidence", 1.0)))
        return clamp01(best)

    def project_lidar_point(self, center_lidar: np.ndarray) -> Optional[Tuple[float, float]]:
        calib = self.calibration
        if calib is None:
            return None
        p_lidar = np.asarray([center_lidar[0], center_lidar[1], center_lidar[2], 1.0], dtype=np.float64)
        p_cam = calib.t_lidar_to_camera @ p_lidar
        z = float(p_cam[2])
        if z <= 1e-3:
            return None
        u = calib.fx * float(p_cam[0]) / z + calib.cx
        v = calib.fy * float(p_cam[1]) / z + calib.cy
        if u < 0.0 or u >= calib.image_width or v < 0.0 or v >= calib.image_height:
            return None
        return u, v

    def temporal_score(self, center: np.ndarray, wall_time: float) -> Tuple[Optional[TemporalTrack], float]:
        best_track = None
        best_dist = float("inf")
        for track in self.tracks:
            age = wall_time - track.last_seen_wall
            if age > self.temporal_track_ttl_sec:
                continue
            dist = float(np.linalg.norm(track.center[:2] - center[:2]))
            if dist < best_dist:
                best_dist = dist
                best_track = track
        if best_track is None or best_dist > self.temporal_match_distance:
            return None, 0.5
        hit_score = min(1.0, 0.45 + 0.18 * float(best_track.hits))
        dist_score = max(0.0, 1.0 - best_dist / max(1e-3, self.temporal_match_distance))
        return best_track, clamp01(0.65 * hit_score + 0.35 * dist_score)

    def update_track(self, center: np.ndarray, matched_track: Optional[TemporalTrack], wall_time: float) -> None:
        if matched_track is not None:
            alpha = self.temporal_smoothing_alpha
            matched_track.center = alpha * center + (1.0 - alpha) * matched_track.center
            matched_track.last_seen_wall = wall_time
            matched_track.hits += 1
            return
        self.tracks.append(TemporalTrack(center=np.asarray(center, dtype=np.float32), last_seen_wall=wall_time))

    def prune_tracks(self, wall_time: float) -> None:
        kept = [track for track in self.tracks if wall_time - track.last_seen_wall <= self.temporal_track_ttl_sec]
        self.tracks = deque(kept, maxlen=self.tracks.maxlen)

    def publish_metrics(
        self,
        msg: ThreeDConeArray,
        start: float,
        *,
        event: str,
        input_count: int,
        accepted_count: int,
        rejected_count: int,
        details: List[Dict[str, Any]],
    ) -> None:
        total_ms = (time.perf_counter() - start) * 1000.0
        accepted_ratio = accepted_count / float(input_count) if input_count else 1.0
        geometry_scores = [item["geometry_score"] for item in details]
        camera_scores = [item["camera_score"] for item in details]
        temporal_scores = [item["temporal_score"] for item in details]
        local_counts = [item["local_point_count"] for item in details]
        payload = {
            "component": "pointpillars_local_verifier",
            "event": event,
            "stamp": stamp_sec(msg),
            "node_stamp": now_sec(self),
            "frame_id": msg.header.frame_id,
            "input_count": input_count,
            "accepted_count": accepted_count,
            "rejected_count": rejected_count,
            "raw_cone_count": input_count,
            "cone_count": accepted_count,
            "accepted_ratio": accepted_ratio,
            "mean_geometry_score": float(np.mean(geometry_scores)) if geometry_scores else None,
            "mean_camera_score": float(np.mean(camera_scores)) if camera_scores else None,
            "mean_temporal_score": float(np.mean(temporal_scores)) if temporal_scores else None,
            "mean_local_point_count": float(np.mean(local_counts)) if local_counts else None,
            "active_tracks": len(self.tracks),
            "total_ms": total_ms,
            "details": details[:12],
        }
        out = String()
        out.data = json.dumps(payload, sort_keys=True)
        self.metrics_pub.publish(out)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PointPillarsLocalVerifier()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
