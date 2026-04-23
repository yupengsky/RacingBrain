#!/usr/bin/env python3
"""Extract a sparse planning-facing track graph from the stable cone map."""

from __future__ import annotations

import json
import math
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


Point2 = Tuple[float, float]


def stamp_to_float(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def marker_color(marker: Marker) -> str:
    r = float(marker.color.r)
    g = float(marker.color.g)
    b = float(marker.color.b)
    if b > 0.7 and r < 0.35 and g < 0.35:
        return "blue"
    if r > 0.7 and g > 0.55 and b < 0.35:
        return "yellow"
    if r > 0.7 and g < 0.45 and b < 0.45:
        return "red"
    return "unknown"


def principal_axis(points: Sequence[Point2]) -> Point2:
    if len(points) < 2:
        return (1.0, 0.0)
    mean_x = sum(p[0] for p in points) / len(points)
    mean_y = sum(p[1] for p in points) / len(points)
    xx = sum((p[0] - mean_x) ** 2 for p in points) / len(points)
    yy = sum((p[1] - mean_y) ** 2 for p in points) / len(points)
    xy = sum((p[0] - mean_x) * (p[1] - mean_y) for p in points) / len(points)
    angle = 0.5 * math.atan2(2.0 * xy, xx - yy)
    return (math.cos(angle), math.sin(angle))


def sort_along_axis(points: Sequence[Point2], axis: Optional[Point2] = None) -> List[Point2]:
    if not points:
        return []
    axis = axis or principal_axis(points)
    ordered = sorted(points, key=lambda p: p[0] * axis[0] + p[1] * axis[1])
    if len(ordered) >= 2 and ordered[0][0] < ordered[-1][0]:
        ordered.reverse()
    return ordered


def to_point(xy: Point2, z: float = 0.08) -> Point:
    point = Point()
    point.x = float(xy[0])
    point.y = float(xy[1])
    point.z = float(z)
    return point


class TrackGraphBuilder(Node):
    def __init__(self) -> None:
        super().__init__("racingbrain_track_graph_builder")
        self.declare_parameter("map_topic", "/global_map")
        self.declare_parameter("max_pair_distance_m", 7.0)
        self.declare_parameter("min_centerline_points", 2)
        self.declare_parameter("max_centerline_points_in_state", 40)

        self.map_topic = str(self.get_parameter("map_topic").value)
        self.max_pair_distance_m = float(self.get_parameter("max_pair_distance_m").value)
        self.min_centerline_points = int(self.get_parameter("min_centerline_points").value)
        self.max_centerline_points_in_state = int(self.get_parameter("max_centerline_points_in_state").value)

        self.track_graph_pub = self.create_publisher(MarkerArray, "/planning/track_graph", 10)
        self.state_pub = self.create_publisher(String, "/racingbrain/planning/input_state", 10)
        self.create_subscription(MarkerArray, self.map_topic, self.cb_map, 10)
        self.get_logger().info(f"Track graph builder listening on {self.map_topic}")

    def cb_map(self, msg: MarkerArray) -> None:
        stamp = self.extract_stamp(msg)
        cones = self.extract_cones(msg)
        left_boundary = [cone["xy"] for cone in cones if cone["side"] == "left"]
        right_boundary = [cone["xy"] for cone in cones if cone["side"] == "right"]
        pairs = self.pair_boundaries(left_boundary, right_boundary)
        centerline = sort_along_axis(
            [((left[0] + right[0]) * 0.5, (left[1] + right[1]) * 0.5) for left, right in pairs]
        )
        ready = len(centerline) >= self.min_centerline_points

        self.publish_track_graph(stamp, left_boundary, right_boundary, centerline, pairs)
        self.publish_state(stamp, left_boundary, right_boundary, centerline, pairs, ready)

    @staticmethod
    def extract_stamp(msg: MarkerArray) -> Optional[Any]:
        for marker in msg.markers:
            return marker.header.stamp
        return None

    @staticmethod
    def extract_cones(msg: MarkerArray) -> List[Dict[str, Any]]:
        cones: List[Dict[str, Any]] = []
        for marker in msg.markers:
            if marker.action == Marker.DELETEALL:
                continue
            color = marker_color(marker)
            if color == "blue":
                side = "left"
            elif color in {"yellow", "red"}:
                side = "right"
            else:
                continue
            cones.append(
                {
                    "xy": (float(marker.pose.position.x), float(marker.pose.position.y)),
                    "side": side,
                    "color": color,
                }
            )
        return cones

    def pair_boundaries(self, left: Sequence[Point2], right: Sequence[Point2]) -> List[Tuple[Point2, Point2]]:
        unused_right = set(range(len(right)))
        pairs: List[Tuple[Point2, Point2]] = []
        axis = principal_axis([*left, *right])
        for left_point in sort_along_axis(left, axis):
            best_index = None
            best_distance = self.max_pair_distance_m
            for index in list(unused_right):
                right_point = right[index]
                distance = math.hypot(left_point[0] - right_point[0], left_point[1] - right_point[1])
                if distance < best_distance:
                    best_distance = distance
                    best_index = index
            if best_index is None:
                continue
            unused_right.remove(best_index)
            pairs.append((left_point, right[best_index]))
        return pairs

    def publish_track_graph(
        self,
        stamp: Optional[Any],
        left_boundary: Sequence[Point2],
        right_boundary: Sequence[Point2],
        centerline: Sequence[Point2],
        pairs: Sequence[Tuple[Point2, Point2]],
    ) -> None:
        msg = MarkerArray()
        msg.markers.append(self.delete_all_marker(stamp))
        all_boundary = [*left_boundary, *right_boundary]
        axis = principal_axis(all_boundary)
        msg.markers.append(self.line_marker(stamp, 1, "left_boundary", sort_along_axis(left_boundary, axis), (0.1, 0.2, 1.0), 0.10))
        msg.markers.append(self.line_marker(stamp, 2, "right_boundary", sort_along_axis(right_boundary, axis), (1.0, 0.85, 0.1), 0.10))
        msg.markers.append(self.line_marker(stamp, 3, "centerline", centerline, (0.1, 1.0, 0.45), 0.16))
        for index, (left, right) in enumerate(pairs[:80], start=10):
            msg.markers.append(self.line_marker(stamp, index, "boundary_pair", [left, right], (1.0, 1.0, 1.0), 0.04, alpha=0.28))
        self.track_graph_pub.publish(msg)

    def delete_all_marker(self, stamp: Optional[Any]) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        if stamp is not None:
            marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker

    def line_marker(
        self,
        stamp: Optional[Any],
        marker_id: int,
        namespace: str,
        points: Sequence[Point2],
        color: Tuple[float, float, float],
        width: float,
        alpha: float = 0.9,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        if stamp is not None:
            marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = float(width)
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(alpha)
        marker.points = [to_point(point) for point in points]
        return marker

    def publish_state(
        self,
        stamp: Optional[Any],
        left_boundary: Sequence[Point2],
        right_boundary: Sequence[Point2],
        centerline: Sequence[Point2],
        pairs: Sequence[Tuple[Point2, Point2]],
        ready: bool,
    ) -> None:
        state = {
            "component": "track_graph_builder",
            "stamp": None if stamp is None else stamp_to_float(stamp),
            "ready": ready,
            "map_topic": self.map_topic,
            "left_boundary_count": len(left_boundary),
            "right_boundary_count": len(right_boundary),
            "paired_boundary_count": len(pairs),
            "centerline_count": len(centerline),
            "centerline_preview_xy": [
                {"x": round(point[0], 3), "y": round(point[1], 3)}
                for point in centerline[: self.max_centerline_points_in_state]
            ],
        }
        msg = String()
        msg.data = json.dumps(state, ensure_ascii=False, separators=(",", ":"))
        self.state_pub.publish(msg)


def main() -> int:
    rclpy.init()
    node = TrackGraphBuilder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
