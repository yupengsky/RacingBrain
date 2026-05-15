from __future__ import annotations

import json
from typing import Dict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from racingbrain.localization.lio_pointcloud_adapter import VelodynePointCloudAdapter


class LioPointCloudAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("lio_pointcloud_adapter")
        self.declare_parameter("input_cloud_topic", "/lidar_points")
        self.declare_parameter("output_cloud_topic", "/points")
        self.declare_parameter("diagnostics_topic", "/racingbrain/localization/lio_pointcloud_adapter")
        self.declare_parameter("output_frame_id", "lidar_link")
        self.declare_parameter("scan_period_sec", 0.1)
        self.declare_parameter("n_scan", 64)
        self.declare_parameter("vertical_fov_lower_deg", -25.0)
        self.declare_parameter("vertical_fov_upper_deg", 15.0)
        self.declare_parameter("missing_ring_policy", "infer")
        self.declare_parameter("missing_time_policy", "azimuth")
        self.declare_parameter("drop_invalid_points", True)

        self.adapter = VelodynePointCloudAdapter(
            output_frame_id=str(self.get_parameter("output_frame_id").value),
            scan_period_sec=float(self.get_parameter("scan_period_sec").value),
            n_scan=int(self.get_parameter("n_scan").value),
            vertical_fov_lower_deg=float(self.get_parameter("vertical_fov_lower_deg").value),
            vertical_fov_upper_deg=float(self.get_parameter("vertical_fov_upper_deg").value),
            missing_ring_policy=str(self.get_parameter("missing_ring_policy").value),
            missing_time_policy=str(self.get_parameter("missing_time_policy").value),
            drop_invalid_points=bool(self.get_parameter("drop_invalid_points").value),
        )
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        input_topic = str(self.get_parameter("input_cloud_topic").value)
        output_topic = str(self.get_parameter("output_cloud_topic").value)
        diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self.publisher = self.create_publisher(PointCloud2, output_topic, qos)
        self.diagnostics_pub = self.create_publisher(String, diagnostics_topic, 10)
        self.create_subscription(PointCloud2, input_topic, self.cloud_callback, qos)
        self.get_logger().info(f"LIO point-cloud adapter ready. input={input_topic}, output={output_topic}")

    def cloud_callback(self, msg: PointCloud2) -> None:
        try:
            adapted, stats = self.adapter.adapt(msg)
        except Exception as exc:  # noqa: BLE001 - report malformed point clouds without killing bag replay.
            self.publish_diagnostics({"component": "lio_pointcloud_adapter", "state": "error", "reason": str(exc)})
            self.get_logger().error(str(exc))
            return
        if adapted is not None:
            self.publisher.publish(adapted)
        payload = {"component": "lio_pointcloud_adapter", "state": "ok", **stats.as_dict()}
        self.publish_diagnostics(payload)

    def publish_diagnostics(self, payload: Dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.diagnostics_pub.publish(msg)


def main() -> int:
    rclpy.init()
    node = LioPointCloudAdapterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
