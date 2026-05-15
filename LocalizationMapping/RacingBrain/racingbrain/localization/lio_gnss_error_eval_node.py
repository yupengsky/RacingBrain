from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Optional

import rclpy
from gnss_ins_msg.msg import Gnssins64
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from racingbrain.localization.lio_gnss_error_eval import ErrorSample, GnssSample, LioGnssErrorEvaluator
from racingbrain.localization.local_tangent import LocalTangentProjector
from racingbrain.localization.pose_sources import Pose2D, wrap_angle
from racingbrain.localization.ros_geometry import stamp_to_float, yaw_from_quaternion


class LioGnssErrorEvalNode(Node):
    def __init__(self) -> None:
        super().__init__("lio_gnss_error_eval")
        self.declare_parameter("gnss_topic", "/gongji_gnss_ins_64")
        self.declare_parameter("lio_odom_topic", "/lio_sam/mapping/odometry")
        self.declare_parameter("diagnostics_topic", "/racingbrain/localization/lio_gnss_error")
        self.declare_parameter("output_dir", "log/benchmark/lio_gnss/latest")
        self.declare_parameter("sync_tolerance_sec", 0.08)
        self.declare_parameter("gnss_queue_sec", 5.0)
        self.declare_parameter("alignment_mode", "first_pair")
        self.declare_parameter("position_warn_m", 1.0)
        self.declare_parameter("yaw_warn_rad", 0.25)
        self.declare_parameter("write_period_sec", 5.0)

        self.projector = LocalTangentProjector()
        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.position_warn_m = float(self.get_parameter("position_warn_m").value)
        self.yaw_warn_rad = float(self.get_parameter("yaw_warn_rad").value)
        self.evaluator = LioGnssErrorEvaluator(
            sync_tolerance_sec=float(self.get_parameter("sync_tolerance_sec").value),
            gnss_queue_sec=float(self.get_parameter("gnss_queue_sec").value),
            alignment_mode=str(self.get_parameter("alignment_mode").value),
        )
        gnss_topic = str(self.get_parameter("gnss_topic").value)
        lio_odom_topic = str(self.get_parameter("lio_odom_topic").value)
        diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        lio_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.diagnostics_pub = self.create_publisher(String, diagnostics_topic, 10)
        self.create_subscription(Gnssins64, gnss_topic, self.gnss_callback, 100)
        self.create_subscription(Odometry, lio_odom_topic, self.lio_callback, lio_qos)
        write_period = max(1.0, float(self.get_parameter("write_period_sec").value))
        self.create_timer(write_period, self.write_outputs)
        self.get_logger().info(
            f"LIO vs GNSS evaluator ready. gnss={gnss_topic}, lio={lio_odom_topic}, output={self.output_dir}"
        )

    def gnss_callback(self, msg: Gnssins64) -> None:
        stamp = stamp_to_float(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self.now_sec()
        x, y = self.projector.forward(float(msg.latitude), float(msg.longitude))
        yaw_ros = wrap_angle(math.radians(float(msg.yaw)) + math.pi * 0.5)
        pose = Pose2D(
            stamp=stamp,
            x=x,
            y=y,
            yaw=yaw_ros,
            vx=float(msg.vel_e),
            vy=float(msg.vel_n),
            yaw_rate=math.radians(float(msg.imu_gyro_z)),
            frame_id="map",
            source="gnss_ins",
        )
        accuracy = float(msg.accuracy_horizon) if float(msg.accuracy_horizon) > 0.0 else None
        self.evaluator.update_gnss(GnssSample(pose=pose, accuracy_xy=accuracy))

    def lio_callback(self, msg: Odometry) -> None:
        stamp = stamp_to_float(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self.now_sec()
        pose = Pose2D(
            stamp=stamp,
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=yaw_from_quaternion(msg.pose.pose.orientation),
            vx=float(msg.twist.twist.linear.x),
            vy=float(msg.twist.twist.linear.y),
            yaw_rate=float(msg.twist.twist.angular.z),
            frame_id=msg.header.frame_id or "odom",
            source="lio",
        )
        sample = self.evaluator.update_lio(pose, covariance_xy=self.extract_xy_covariance(msg))
        if sample is None:
            return
        self.publish_diagnostics(sample)

    def publish_diagnostics(self, sample: ErrorSample) -> None:
        msg = String()
        payload = {
            "component": "lio_gnss_error_eval",
            "state": "ok",
            "sample_count": len(self.evaluator.samples),
            "position_error_m": sample.position_error_m,
            "yaw_error_rad": sample.yaw_error_rad,
            "time_offset_sec": sample.time_offset_sec,
            "alignment_initialized": self.evaluator.aligned,
        }
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.diagnostics_pub.publish(msg)

    def write_outputs(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        rows = [sample.as_row() for sample in self.evaluator.samples]
        if rows:
            csv_path = self.output_dir / "lio_gnss_error_trace.csv"
            with csv_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
                writer.writeheader()
                writer.writerows(rows)
        summary = self.evaluator.summary(position_warn_m=self.position_warn_m, yaw_warn_rad=self.yaw_warn_rad)
        (self.output_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        lines = [
            "# LIO-SAM vs GNSS/INS Evaluation",
            "",
            f"- Samples: `{summary['sample_count']}`",
            f"- Alignment mode: `{summary['alignment_mode']}`",
            f"- Alignment initialized: `{summary['alignment_initialized']}`",
            f"- Position error (m): `{summary['position_error_m']}`",
            f"- Yaw error (rad): `{summary['yaw_error_rad']}`",
            f"- Abs time offset (s): `{summary['abs_time_offset_sec']}`",
            f"- Position warnings: `{summary['position_warn_count']}` >= `{summary['position_warn_m']}` m",
            f"- Yaw warnings: `{summary['yaw_warn_count']}` >= `{summary['yaw_warn_rad']}` rad",
        ]
        (self.output_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    @staticmethod
    def extract_xy_covariance(msg: Odometry) -> Optional[float]:
        values = [float(msg.pose.covariance[0]), float(msg.pose.covariance[7])]
        values = [value for value in values if value > 0.0 and math.isfinite(value)]
        return max(values) if values else None

    def now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9


def main() -> int:
    rclpy.init()
    node = LioGnssErrorEvalNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.write_outputs()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
