#!/usr/bin/env python3
"""Convert FSDS testing-only track ground truth into `drd25_msgs/Map`."""

from __future__ import annotations

from typing import Any, Iterable, Optional, Tuple

import rclpy
from drd25_msgs.msg import Cone, Map
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def cone_xy(cone: Any) -> Optional[Tuple[float, float]]:
    for attr in ("location", "position", "point", "center"):
        value = getattr(cone, attr, None)
        if value is not None and hasattr(value, "x") and hasattr(value, "y"):
            return float(value.x), float(value.y)
    if hasattr(cone, "x") and hasattr(cone, "y"):
        return float(cone.x), float(cone.y)
    return None


def iter_track_cones(track_msg: Any) -> Iterable[Tuple[Any, str]]:
    for attr in (
        "track",
        "cones",
        "blue_cones",
        "yellow_cones",
        "orange_cones",
        "big_orange_cones",
        "left_cones",
        "right_cones",
        "unknown_cones",
    ):
        cones = getattr(track_msg, attr, None)
        if cones:
            for cone in cones:
                yield cone, attr


def color_id(cone: Any, source_attr: str) -> int:
    schema = Cone()
    blue = getattr(schema, "BLUE", 0)
    right = getattr(schema, "YELLOW", getattr(schema, "RED", 1))
    orange_big = getattr(schema, "ORANGE_BIG", getattr(schema, "YELLOW_BIG", right))
    orange_small = getattr(schema, "ORANGE_SMALL", getattr(schema, "YELLOW_SMALL", right))
    unknown = getattr(schema, "UNKNOWN", 4)

    source_attr = source_attr.lower()
    if "blue" in source_attr or "left" in source_attr:
        return int(blue)
    if "yellow" in source_attr or "right" in source_attr:
        return int(right)
    if "big_orange" in source_attr:
        return int(orange_big)
    if "orange" in source_attr:
        return int(orange_small)

    raw = getattr(cone, "color", None)
    if raw is None:
        return int(unknown)
    if isinstance(raw, str):
        raw_lower = raw.lower()
        if "blue" in raw_lower:
            return int(blue)
        if "yellow" in raw_lower or "red" in raw_lower:
            return int(right)
        if "orange" in raw_lower and "big" in raw_lower:
            return int(orange_big)
        if "orange" in raw_lower:
            return int(orange_small)
        return int(unknown)
    try:
        return int(raw)
    except (TypeError, ValueError):
        return int(unknown)


def param_as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


class FSDSTrackToMapAdapter(Node):
    def __init__(self) -> None:
        super().__init__("fsds_track_to_map_adapter")
        self.declare_parameter("fsds_track_topic", "/testing_only/track")
        self.declare_parameter("planner_map_topic", "/drd25/map")
        self.declare_parameter("fsds_track_transient_local", False)

        try:
            from fs_msgs.msg import Track  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "fs_msgs is required for fsds_track_to_map_adapter. "
                "Source or build the FSDS ROS/ROS2 workspace first."
            ) from exc

        fsds_track_topic = str(self.get_parameter("fsds_track_topic").value)
        planner_map_topic = str(self.get_parameter("planner_map_topic").value)
        fsds_track_transient_local = param_as_bool(
            self.get_parameter("fsds_track_transient_local").value
        )
        latched_map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        track_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
            if fsds_track_transient_local
            else DurabilityPolicy.VOLATILE,
        )
        self.map_pub = self.create_publisher(Map, planner_map_topic, latched_map_qos)
        self.create_subscription(Track, fsds_track_topic, self.cb_track, track_qos)
        self.get_logger().info(f"Adapting FSDS track {fsds_track_topic} -> {planner_map_topic}")

    def cb_track(self, msg: Any) -> None:
        out = Map()
        header = getattr(msg, "header", None)
        if header is not None:
            out.header = header
        else:
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = "map"

        for fsds_cone, source_attr in iter_track_cones(msg):
            xy = cone_xy(fsds_cone)
            if xy is None:
                continue
            cone = Cone()
            cone.x = xy[0]
            cone.y = xy[1]
            cone.color = color_id(fsds_cone, source_attr)
            out.track.append(cone)
        self.map_pub.publish(out)


def main() -> int:
    rclpy.init()
    node = FSDSTrackToMapAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
