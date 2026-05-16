from __future__ import annotations

import json
import math
from collections import deque
from typing import Dict, Optional

import rclpy
from gnss_ins_msg.msg import Gnssins64
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String

from racingbrain.localization.local_tangent import LocalTangentProjector
from racingbrain.localization.pose_sources import wrap_angle
from racingbrain.localization.ros_geometry import quaternion_from_yaw, stamp_to_float
from racingbrain.localization.simple_lio_core import (
    IcpResult,
    Keyframe,
    OnlineImuYawBuffer,
    Pose2,
    SimpleLioConfig,
    blend_pose,
    build_local_map,
    clamp_motion,
    extract_scan_2d,
    predict_pose,
    run_icp,
    should_add_keyframe,
    transform_points,
    voxel_downsample,
)


class SimpleLioNode(Node):
    def __init__(self) -> None:
        super().__init__("simple_lio")
        self.declare_parameter("cloud_topic", "/lidar_points")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("gnss_topic", "/gongji_gnss_ins_64")
        self.declare_parameter("odom_topic", "/racingbrain/simple_lio/odometry")
        self.declare_parameter("diagnostics_topic", "/racingbrain/localization/simple_lio")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("init_gnss_max_age_sec", 2.0)
        self.declare_parameter("imu_buffer_sec", 30.0)

        for field, default in SimpleLioConfig().__dict__.items():
            self.declare_parameter(field, default)

        self.cfg = self.config_from_parameters()
        self.projector = LocalTangentProjector()
        self.imu = OnlineImuYawBuffer(
            gyro_scale=self.cfg.imu_gyro_scale,
            max_age_sec=float(self.get_parameter("imu_buffer_sec").value),
        )
        self.keyframes = deque()
        self.latest_gnss_pose: Optional[Pose2] = None
        self.previous_pose: Optional[Pose2] = None
        self.previous_previous_pose: Optional[Pose2] = None
        self.cloud_count = 0
        self.skipped_cloud_count = 0
        self.icp_used_count = 0

        sensor_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_pub = self.create_publisher(Odometry, str(self.get_parameter("odom_topic").value), 20)
        self.diagnostics_pub = self.create_publisher(String, str(self.get_parameter("diagnostics_topic").value), 10)
        self.create_subscription(PointCloud2, str(self.get_parameter("cloud_topic").value), self.cloud_callback, sensor_qos)
        self.create_subscription(Imu, str(self.get_parameter("imu_topic").value), self.imu_callback, sensor_qos)
        self.create_subscription(Gnssins64, str(self.get_parameter("gnss_topic").value), self.gnss_callback, 100)

        self.get_logger().info(
            "Simple LIO ready. "
            f"cloud={self.get_parameter('cloud_topic').value}, "
            f"imu={self.get_parameter('imu_topic').value}, "
            f"gnss={self.get_parameter('gnss_topic').value}, "
            f"odom={self.get_parameter('odom_topic').value}"
        )

    def config_from_parameters(self) -> SimpleLioConfig:
        values = {}
        for field in SimpleLioConfig().__dict__:
            values[field] = self.get_parameter(field).value
        return SimpleLioConfig(**values)

    def gnss_callback(self, msg: Gnssins64) -> None:
        stamp = stamp_to_float(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self.now_sec()
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        if not (math.isfinite(lat) and math.isfinite(lon)) or (abs(lat) < 1e-12 and abs(lon) < 1e-12):
            return
        x, y = self.projector.forward(lat, lon)
        yaw = wrap_angle(math.radians(float(msg.yaw)) + math.pi * 0.5)
        self.latest_gnss_pose = Pose2(
            stamp=stamp,
            x=x,
            y=y,
            yaw=yaw,
            vx=float(msg.vel_e),
            vy=float(msg.vel_n),
        )

    def imu_callback(self, msg: Imu) -> None:
        stamp = stamp_to_float(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self.now_sec()
        self.imu.add(stamp, float(msg.angular_velocity.z))

    def cloud_callback(self, msg: PointCloud2) -> None:
        stamp = stamp_to_float(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self.now_sec()
        try:
            scan = extract_scan_2d(msg, self.cfg)
        except Exception as exc:  # noqa: BLE001 - malformed PointCloud2 should not kill bag replay.
            self.skipped_cloud_count += 1
            self.publish_diagnostics({"state": "error", "reason": str(exc), "stamp": stamp})
            return

        if len(scan) < self.cfg.icp_min_inliers:
            self.skipped_cloud_count += 1
            self.publish_diagnostics(
                {
                    "state": "waiting",
                    "reason": "scan_too_sparse",
                    "stamp": stamp,
                    "scan_points": int(len(scan)),
                }
            )
            return

        if self.previous_pose is None:
            pose, icp = self.initialize_pose(stamp)
            if pose is None:
                self.skipped_cloud_count += 1
                self.publish_diagnostics(
                    {
                        "state": "waiting",
                        "reason": "waiting_for_gnss_ins_initialization",
                        "stamp": stamp,
                        "scan_points": int(len(scan)),
                    }
                )
                return
        else:
            predicted = predict_pose(self.previous_pose, self.previous_previous_pose, stamp, self.imu, self.cfg)
            local_map = build_local_map(self.keyframes, self.cfg)
            icp = run_icp(scan, local_map, predicted, self.cfg)
            pose = icp.pose
            if icp.used:
                self.icp_used_count += 1
            if icp.used and len(scan) < self.cfg.sparse_scan_points:
                pose = blend_pose(predicted, pose, self.cfg.sparse_icp_weight)
                icp.pose = pose
                icp.reason = f"{icp.reason}_sparse_scan_blend"
            pose, icp = self.finish_motion_update(pose, icp, scan_points=len(scan))

        if should_add_keyframe(self.keyframes, pose, self.cfg):
            world_points = voxel_downsample(transform_points(scan, pose), self.cfg.map_voxel_m, self.cfg.max_keyframe_points)
            self.keyframes.append(Keyframe(stamp=pose.stamp, x=pose.x, y=pose.y, yaw=pose.yaw, points_world=world_points))
            while len(self.keyframes) > self.cfg.local_map_keyframes:
                self.keyframes.popleft()

        self.previous_previous_pose = self.previous_pose
        self.previous_pose = pose
        self.cloud_count += 1
        self.publish_odometry(msg, pose, icp)
        self.publish_diagnostics(
            {
                "state": "ok",
                "reason": icp.reason,
                "stamp": stamp,
                "processed_clouds": self.cloud_count,
                "skipped_clouds": self.skipped_cloud_count,
                "scan_points": int(len(scan)),
                "keyframes": len(self.keyframes),
                "icp_used": icp.used,
                "icp_used_count": self.icp_used_count,
                "icp_inliers": icp.inliers,
                "icp_rmse_m": icp.rmse,
                "icp_median_error_m": icp.median_error,
                "icp_iterations": icp.iterations,
                "x": pose.x,
                "y": pose.y,
                "yaw": pose.yaw,
            }
        )

    def initialize_pose(self, stamp: float) -> tuple[Optional[Pose2], IcpResult]:
        if self.latest_gnss_pose is None:
            empty = Pose2(stamp=stamp, x=0.0, y=0.0, yaw=0.0)
            return None, IcpResult(empty, False, 0, None, None, 0, "waiting_for_gnss_ins_initialization")
        max_age = float(self.get_parameter("init_gnss_max_age_sec").value)
        if abs(stamp - self.latest_gnss_pose.stamp) > max_age:
            return None, IcpResult(
                self.latest_gnss_pose.copy_at(stamp),
                False,
                0,
                None,
                None,
                0,
                "gnss_ins_initialization_stale",
            )
        pose = Pose2(
            stamp=stamp,
            x=self.latest_gnss_pose.x,
            y=self.latest_gnss_pose.y,
            yaw=self.latest_gnss_pose.yaw,
            vx=self.latest_gnss_pose.vx,
            vy=self.latest_gnss_pose.vy,
        )
        return pose, IcpResult(pose, False, 0, None, None, 0, "initial_gnss_ins_alignment")

    def finish_motion_update(self, pose: Pose2, icp: IcpResult, *, scan_points: int) -> tuple[Pose2, IcpResult]:
        assert self.previous_pose is not None
        pose, motion_clamped = clamp_motion(self.previous_pose, pose, self.cfg)
        if motion_clamped:
            icp.reason = f"{icp.reason}_motion_clamped"
            icp.pose = pose
        dt = pose.stamp - self.previous_pose.stamp
        if 1e-3 < dt <= self.cfg.max_frame_dt_sec:
            observed_vx = (pose.x - self.previous_pose.x) / dt
            observed_vy = (pose.y - self.previous_pose.y) / dt
            alpha = self.cfg.sparse_velocity_update_alpha if scan_points < self.cfg.sparse_scan_points else self.cfg.velocity_update_alpha
            pose.vx = self.previous_pose.vx * (1.0 - alpha) + observed_vx * alpha
            pose.vy = self.previous_pose.vy * (1.0 - alpha) + observed_vy * alpha
        else:
            pose.vx = self.previous_pose.vx
            pose.vy = self.previous_pose.vy
        return pose, icp

    def publish_odometry(self, cloud_msg: PointCloud2, pose: Pose2, icp: IcpResult) -> None:
        msg = Odometry()
        msg.header.stamp = cloud_msg.header.stamp
        msg.header.frame_id = str(self.get_parameter("map_frame").value)
        msg.child_frame_id = str(self.get_parameter("base_frame").value)
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = quaternion_from_yaw(pose.yaw)
        msg.twist.twist.linear.x = pose.vx
        msg.twist.twist.linear.y = pose.vy

        covariance_xy = self.estimate_xy_covariance(icp)
        covariance_yaw = self.estimate_yaw_covariance(icp)
        msg.pose.covariance[0] = covariance_xy
        msg.pose.covariance[7] = covariance_xy
        msg.pose.covariance[35] = covariance_yaw
        self.odom_pub.publish(msg)

    @staticmethod
    def estimate_xy_covariance(icp: IcpResult) -> float:
        if icp.used and icp.rmse is not None:
            return float(max(0.02, min(5.0, icp.rmse * icp.rmse)))
        if icp.reason == "initial_gnss_ins_alignment":
            return 0.04
        return 0.75

    @staticmethod
    def estimate_yaw_covariance(icp: IcpResult) -> float:
        if icp.used:
            return 0.0025
        if icp.reason == "initial_gnss_ins_alignment":
            return 0.01
        return 0.08

    def publish_diagnostics(self, payload: Dict[str, object]) -> None:
        payload = {"component": "simple_lio", **payload}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.diagnostics_pub.publish(msg)

    def now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9


def main() -> int:
    rclpy.init()
    node = SimpleLioNode()
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

