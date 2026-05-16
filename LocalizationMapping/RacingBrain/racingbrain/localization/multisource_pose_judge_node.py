from __future__ import annotations

import json
import math
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Quaternion
from gnss_ins_msg.msg import Gnssins64
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from racingbrain.localization.pose_sources import (
    MultiSourcePoseJudge,
    Pose2D,
    PoseDecision,
    PoseJudgeConfig,
    wrap_angle,
)


EARTH_RADIUS_M = 6378137.0


def stamp_to_float(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    half = 0.5 * yaw
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class LocalTangentProjector:
    def __init__(self) -> None:
        self.origin_lat_deg: Optional[float] = None
        self.origin_lon_deg: Optional[float] = None
        self.origin_lat_rad: Optional[float] = None

    @property
    def ready(self) -> bool:
        return self.origin_lat_deg is not None and self.origin_lon_deg is not None

    def ensure_origin(self, lat_deg: float, lon_deg: float) -> None:
        if self.ready:
            return
        self.origin_lat_deg = float(lat_deg)
        self.origin_lon_deg = float(lon_deg)
        self.origin_lat_rad = math.radians(float(lat_deg))

    def forward(self, lat_deg: float, lon_deg: float) -> tuple[float, float]:
        self.ensure_origin(lat_deg, lon_deg)
        assert self.origin_lat_deg is not None
        assert self.origin_lon_deg is not None
        assert self.origin_lat_rad is not None
        x = math.radians(lon_deg - self.origin_lon_deg) * math.cos(self.origin_lat_rad) * EARTH_RADIUS_M
        y = math.radians(lat_deg - self.origin_lat_deg) * EARTH_RADIUS_M
        return x, y

    def inverse(self, x: float, y: float) -> tuple[float, float]:
        if not self.ready:
            raise RuntimeError("local tangent origin has not been initialized")
        assert self.origin_lat_deg is not None
        assert self.origin_lon_deg is not None
        assert self.origin_lat_rad is not None
        lat = self.origin_lat_deg + math.degrees(y / EARTH_RADIUS_M)
        lon = self.origin_lon_deg + math.degrees(x / (EARTH_RADIUS_M * math.cos(self.origin_lat_rad)))
        return lat, lon


class MultiSourcePoseJudgeNode(Node):
    def __init__(self) -> None:
        super().__init__("multisource_pose_judge")

        self.declare_parameter("gnss_topic", "/gongji_gnss_ins_64")
        self.declare_parameter("lio_odom_topic", "/racingbrain/simple_lio/odometry")
        self.declare_parameter("output_odom_topic", "/racingbrain/localization/pose")
        self.declare_parameter("output_gnss_topic", "/racingbrain/localization/gnss_ins_pose")
        self.declare_parameter("diagnostics_topic", "/racingbrain/localization/pose_judge")
        self.declare_parameter("publish_compat_gnss", True)
        self.declare_parameter("publish_period_sec", 0.05)
        self.declare_parameter("stale_timeout_sec", 0.35)
        self.declare_parameter("fusion_enabled", True)
        self.declare_parameter("gnss_accuracy_good_m", 0.25)
        self.declare_parameter("gnss_accuracy_warn_m", 1.0)
        self.declare_parameter("gnss_accuracy_bad_m", 2.5)
        self.declare_parameter("lio_cov_good_m2", 0.08)
        self.declare_parameter("lio_cov_warn_m2", 0.55)
        self.declare_parameter("lio_cov_bad_m2", 2.0)
        self.declare_parameter("cross_position_warn_m", 0.75)
        self.declare_parameter("cross_position_reject_m", 2.0)
        self.declare_parameter("cross_yaw_warn_rad", 0.18)
        self.declare_parameter("cross_yaw_reject_rad", 0.55)

        config = PoseJudgeConfig(
            stale_timeout_sec=float(self.get_parameter("stale_timeout_sec").value),
            fusion_enabled=bool(self.get_parameter("fusion_enabled").value),
            gnss_accuracy_good_m=float(self.get_parameter("gnss_accuracy_good_m").value),
            gnss_accuracy_warn_m=float(self.get_parameter("gnss_accuracy_warn_m").value),
            gnss_accuracy_bad_m=float(self.get_parameter("gnss_accuracy_bad_m").value),
            lio_cov_good_m2=float(self.get_parameter("lio_cov_good_m2").value),
            lio_cov_warn_m2=float(self.get_parameter("lio_cov_warn_m2").value),
            lio_cov_bad_m2=float(self.get_parameter("lio_cov_bad_m2").value),
            cross_position_warn_m=float(self.get_parameter("cross_position_warn_m").value),
            cross_position_reject_m=float(self.get_parameter("cross_position_reject_m").value),
            cross_yaw_warn_rad=float(self.get_parameter("cross_yaw_warn_rad").value),
            cross_yaw_reject_rad=float(self.get_parameter("cross_yaw_reject_rad").value),
        )
        self.judge = MultiSourcePoseJudge(config)
        self.projector = LocalTangentProjector()
        self.last_gnss_msg: Optional[Gnssins64] = None
        self.last_decision_stamp: Optional[float] = None

        gnss_topic = str(self.get_parameter("gnss_topic").value)
        lio_odom_topic = str(self.get_parameter("lio_odom_topic").value)
        output_odom_topic = str(self.get_parameter("output_odom_topic").value)
        output_gnss_topic = str(self.get_parameter("output_gnss_topic").value)
        diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        publish_period_sec = max(0.01, float(self.get_parameter("publish_period_sec").value))

        self.odom_pub = self.create_publisher(Odometry, output_odom_topic, 10)
        self.diag_pub = self.create_publisher(String, diagnostics_topic, 10)
        self.gnss_pub = self.create_publisher(Gnssins64, output_gnss_topic, 10)
        self.create_subscription(Gnssins64, gnss_topic, self.cb_gnss, 50)
        self.create_subscription(Odometry, lio_odom_topic, self.cb_lio, 50)
        self.create_timer(publish_period_sec, self.publish_decision)

        self.get_logger().info(
            "Multi-source pose judge ready. "
            f"gnss={gnss_topic}, lio={lio_odom_topic}, "
            f"odom_out={output_odom_topic}, gnss_out={output_gnss_topic}"
        )

    def cb_gnss(self, msg: Gnssins64) -> None:
        self.last_gnss_msg = msg
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
        )
        accuracy = float(msg.accuracy_horizon) if float(msg.accuracy_horizon) > 0.0 else None
        accuracy_yaw = math.radians(float(msg.accuracy_yaw)) if float(msg.accuracy_yaw) > 0.0 else None
        self.judge.update_gnss(
            pose,
            accuracy_xy=accuracy,
            accuracy_yaw=accuracy_yaw,
            status=int(msg.gnss_status),
        )

    def cb_lio(self, msg: Odometry) -> None:
        stamp = stamp_to_float(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self.now_sec()
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        covariance_xy = self.extract_xy_covariance(msg)
        covariance_yaw = self.extract_yaw_covariance(msg)
        pose = Pose2D(
            stamp=stamp,
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=yaw,
            vx=float(msg.twist.twist.linear.x),
            vy=float(msg.twist.twist.linear.y),
            yaw_rate=float(msg.twist.twist.angular.z),
            frame_id=msg.header.frame_id or "odom",
        )
        self.judge.update_lio(pose, covariance_xy=covariance_xy, covariance_yaw=covariance_yaw)

    def publish_decision(self) -> None:
        decision = self.judge.decide(self.now_sec())
        if decision.pose is None:
            self.publish_diagnostics(decision)
            return
        if self.last_decision_stamp is not None and decision.pose.stamp <= self.last_decision_stamp:
            self.publish_diagnostics(decision)
            return
        self.last_decision_stamp = decision.pose.stamp

        self.odom_pub.publish(self.to_odometry(decision))
        if bool(self.get_parameter("publish_compat_gnss").value) and self.projector.ready:
            self.gnss_pub.publish(self.to_gnss_compat(decision))
        self.publish_diagnostics(decision)

    def publish_diagnostics(self, decision: PoseDecision) -> None:
        payload = decision.as_dict()
        payload["component"] = "multisource_pose_judge"
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.diag_pub.publish(msg)

    def to_odometry(self, decision: PoseDecision) -> Odometry:
        assert decision.pose is not None
        pose = decision.pose
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = pose.frame_id or "map"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = quaternion_from_yaw(pose.yaw)
        msg.twist.twist.linear.x = pose.vx
        msg.twist.twist.linear.y = pose.vy
        msg.twist.twist.angular.z = pose.yaw_rate
        return msg

    def to_gnss_compat(self, decision: PoseDecision) -> Gnssins64:
        assert decision.pose is not None
        pose = decision.pose
        if decision.source == "gnss_ins" and self.last_gnss_msg is not None:
            return self.copy_gnss_msg(self.last_gnss_msg)

        lat, lon = self.projector.inverse(pose.x, pose.y)
        msg = self.copy_gnss_msg(self.last_gnss_msg) if self.last_gnss_msg is not None else Gnssins64()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.height = 0.0
        msg.vel_e = float(pose.vx)
        msg.vel_n = float(pose.vy)
        msg.vel_u = 0.0
        msg.pitch = 0.0
        msg.roll = 0.0
        msg.yaw = math.degrees(wrap_angle(pose.yaw - math.pi * 0.5))
        msg.imu_gyro_z = math.degrees(float(pose.yaw_rate))
        msg.accuracy_horizon = self.compat_accuracy(decision)
        msg.accuracy_yaw = 0.0
        return msg

    @staticmethod
    def copy_gnss_msg(src: Optional[Gnssins64]) -> Gnssins64:
        dst = Gnssins64()
        if src is None:
            return dst
        dst.header = src.header
        dst.latitude = src.latitude
        dst.longitude = src.longitude
        dst.height = src.height
        dst.vel_e = src.vel_e
        dst.vel_n = src.vel_n
        dst.vel_u = src.vel_u
        dst.pitch = src.pitch
        dst.roll = src.roll
        dst.yaw = src.yaw
        dst.ins_status = src.ins_status
        dst.gnss_week = src.gnss_week
        dst.gnss_second = src.gnss_second
        dst.gnss_status = src.gnss_status
        dst.satellite_main = src.satellite_main
        dst.satellite_sub = src.satellite_sub
        dst.imu_acc_x = src.imu_acc_x
        dst.imu_acc_y = src.imu_acc_y
        dst.imu_acc_z = src.imu_acc_z
        dst.imu_gyro_x = src.imu_gyro_x
        dst.imu_gyro_y = src.imu_gyro_y
        dst.imu_gyro_z = src.imu_gyro_z
        dst.imu_temp = src.imu_temp
        dst.accuracy_horizon = src.accuracy_horizon
        dst.accuracy_height = src.accuracy_height
        dst.accuracy_horizon_velocity = src.accuracy_horizon_velocity
        dst.accuracy_vertical_velocity = src.accuracy_vertical_velocity
        dst.accuracy_horizon_posture = src.accuracy_horizon_posture
        dst.accuracy_yaw = src.accuracy_yaw
        return dst

    @staticmethod
    def compat_accuracy(decision: PoseDecision) -> float:
        if decision.cross_position_error_m is None:
            return 0.5 if decision.source == "lio" else 0.25
        return float(max(0.2, min(5.0, decision.cross_position_error_m)))

    @staticmethod
    def extract_xy_covariance(msg: Odometry) -> Optional[float]:
        values = [float(msg.pose.covariance[0]), float(msg.pose.covariance[7])]
        values = [value for value in values if value > 0.0 and math.isfinite(value)]
        if not values:
            return None
        return max(values)

    @staticmethod
    def extract_yaw_covariance(msg: Odometry) -> Optional[float]:
        value = float(msg.pose.covariance[35])
        if value <= 0.0 or not math.isfinite(value):
            return None
        return value

    def now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9


def main() -> int:
    rclpy.init()
    node = MultiSourcePoseJudgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
