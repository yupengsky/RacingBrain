from __future__ import annotations

import json
import math
from typing import Dict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from racingbrain.localization.lio_imu_adapter import LioImuAdapter


class LioImuAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("lio_imu_adapter")
        self.declare_parameter("input_imu_topic", "/imu")
        self.declare_parameter("output_imu_topic", "/imu_lio")
        self.declare_parameter("diagnostics_topic", "/racingbrain/localization/lio_imu_adapter")
        self.declare_parameter("acceleration_scale", 9.80665)
        self.declare_parameter("gyro_scale", math.pi / 180.0)

        self.adapter = LioImuAdapter(
            acceleration_scale=float(self.get_parameter("acceleration_scale").value),
            gyro_scale=float(self.get_parameter("gyro_scale").value),
        )
        self.count = 0
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        input_topic = str(self.get_parameter("input_imu_topic").value)
        output_topic = str(self.get_parameter("output_imu_topic").value)
        diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self.publisher = self.create_publisher(Imu, output_topic, qos)
        self.diagnostics_pub = self.create_publisher(String, diagnostics_topic, 10)
        self.create_subscription(Imu, input_topic, self.imu_callback, qos)
        self.get_logger().info(
            "LIO IMU adapter ready. "
            f"input={input_topic}, output={output_topic}, "
            f"acc_scale={self.adapter.acceleration_scale}, gyro_scale={self.adapter.gyro_scale}"
        )

    def imu_callback(self, msg: Imu) -> None:
        adapted = self.adapter.adapt(msg)
        self.publisher.publish(adapted)
        self.count += 1
        if self.count == 1 or self.count % 500 == 0:
            self.publish_diagnostics(
                {
                    "component": "lio_imu_adapter",
                    "state": "ok",
                    "message_count": self.count,
                    "acceleration_scale": self.adapter.acceleration_scale,
                    "gyro_scale": self.adapter.gyro_scale,
                    "sample_accel_z": adapted.linear_acceleration.z,
                    "sample_gyro_z": adapted.angular_velocity.z,
                }
            )

    def publish_diagnostics(self, payload: Dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.diagnostics_pub.publish(msg)


def main() -> int:
    rclpy.init()
    node = LioImuAdapterNode()
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
