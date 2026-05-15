from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from drd25_msgs.msg import Cone, Map
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from test_cone_segmentation.msg import ThreeDConeArray


def now_sec(node: Node) -> float:
    return float(node.get_clock().now().nanoseconds) * 1e-9


class LidarConesToMap(Node):
    """Adapt LiDAR-only cone detections to the mapper's fused-map interface."""

    def __init__(self) -> None:
        super().__init__("lidar_cones_to_map")

        self.declare_parameter("input_topic", "/cone_detection_custom")
        self.declare_parameter("output_topic", "/perception/fusion/map")
        self.declare_parameter("metrics_topic", "/perception/fusion/evaluation/metrics")
        self.declare_parameter("color_policy", "y_sign")
        self.declare_parameter("min_range_m", 0.0)
        self.declare_parameter("max_range_m", 30.0)
        self.declare_parameter("max_abs_y_m", 12.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        metrics_topic = str(self.get_parameter("metrics_topic").value)
        self.color_policy = str(self.get_parameter("color_policy").value).strip().lower()
        self.min_range_m = max(0.0, float(self.get_parameter("min_range_m").value))
        self.max_range_m = max(self.min_range_m, float(self.get_parameter("max_range_m").value))
        self.max_abs_y_m = max(0.0, float(self.get_parameter("max_abs_y_m").value))

        self.map_pub = self.create_publisher(Map, output_topic, 10)
        self.metrics_pub = self.create_publisher(String, metrics_topic, 10)
        self.create_subscription(ThreeDConeArray, input_topic, self.cb_cones, 10)

        self.get_logger().info(
            "LiDAR-only fusion bridge ready. "
            f"{input_topic} -> {output_topic}, color_policy={self.color_policy}"
        )

    def cb_cones(self, msg: ThreeDConeArray) -> None:
        started = time.perf_counter()
        out = Map()
        out.header = msg.header

        dropped = 0
        for lidar_cone in msg.cones:
            x = float(lidar_cone.center.x)
            y = float(lidar_cone.center.y)
            if not self.accept_point(x, y):
                dropped += 1
                continue

            cone = Cone()
            cone.x = x
            cone.y = y
            cone.color = self.color_for_y(y)
            out.track.append(cone)

        self.map_pub.publish(out)
        self.publish_metrics(msg, out, dropped, (time.perf_counter() - started) * 1000.0)

    def accept_point(self, x: float, y: float) -> bool:
        if not (math.isfinite(x) and math.isfinite(y)):
            return False
        distance = math.hypot(x, y)
        if distance < self.min_range_m or distance > self.max_range_m:
            return False
        return abs(y) <= self.max_abs_y_m

    def color_for_y(self, y: float) -> int:
        if self.color_policy == "single_blue":
            return int(Cone.BLUE)
        if self.color_policy == "single_yellow":
            return int(getattr(Cone, "YELLOW", getattr(Cone, "YELLOW_BIG", Cone.BLUE)))
        if self.color_policy == "single_red":
            return int(getattr(Cone, "RED", getattr(Cone, "YELLOW", Cone.BLUE)))

        left_color = int(Cone.BLUE)
        right_color = int(getattr(Cone, "RED", getattr(Cone, "YELLOW", Cone.BLUE)))
        return left_color if y >= 0.0 else right_color

    def publish_metrics(
        self,
        source: ThreeDConeArray,
        out: Map,
        dropped: int,
        total_ms: float,
    ) -> None:
        payload = {
            "component": "lidar_only_fusion_bridge",
            "event": "processed",
            "stamp": now_sec(self),
            "source_stamp_sec": source.header.stamp.sec,
            "source_stamp_nanosec": source.header.stamp.nanosec,
            "input_count": len(source.cones),
            "final_count": len(out.track),
            "unknown_count": 0,
            "dropped_count": dropped,
            "recovered_count": 0,
            "force_match_count": 0,
            "valid_projection_ratio": 1.0,
            "low_iou_ratio": 0.0,
            "consistency_score": 1.0,
            "calibration_drift_score": 0.0,
            "abs_camera_lidar_stamp_delta_ms": 0.0,
            "total_ms": total_ms,
            "mode": "lidar_only",
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.metrics_pub.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LidarConesToMap()
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
