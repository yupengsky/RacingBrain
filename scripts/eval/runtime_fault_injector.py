#!/usr/bin/env python3
"""Replay-only fault injector for RacingBrain sensor topics.

The injector subscribes to the original rosbag topics and republishes them to
shadow topics consumed by the stack under test. It is intentionally lightweight
and only used by offline reliability experiments.
"""

from __future__ import annotations

import argparse
import copy
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from gnss_ins_msg.msg import Gnssins64
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2


def clamp_kernel(kernel: int) -> int:
    kernel = max(1, int(kernel))
    if kernel % 2 == 0:
        kernel += 1
    return kernel


def shift_stamp(stamp: Any, offset_sec: float) -> None:
    total = float(stamp.sec) + float(stamp.nanosec) * 1e-9 + float(offset_sec)
    if total < 0.0:
        total = 0.0
    sec = int(total)
    nanosec = int(round((total - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    stamp.sec = sec
    stamp.nanosec = nanosec


@dataclass
class ChannelFault:
    mode: str
    drop_every: int
    stamp_offset_sec: float
    publish_delay_sec: float
    blur_kernel: int = 15


class RuntimeFaultInjector(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("runtime_fault_injector")
        self.bridge = CvBridge()
        self.start_wall = time.monotonic()
        self.fault_start_sec = max(0.0, float(args.fault_start_sec))
        self.fault_duration_sec = float(args.fault_duration_sec)
        self.log_path = Path(args.log_path) if args.log_path else None

        self.camera_fault = ChannelFault(
            mode=args.camera_mode,
            drop_every=max(1, int(args.camera_drop_every)),
            stamp_offset_sec=float(args.camera_stamp_offset_sec),
            publish_delay_sec=max(0.0, float(args.camera_publish_delay_sec)),
            blur_kernel=clamp_kernel(args.camera_blur_kernel),
        )
        self.lidar_fault = ChannelFault(
            mode=args.lidar_mode,
            drop_every=max(1, int(args.lidar_drop_every)),
            stamp_offset_sec=float(args.lidar_stamp_offset_sec),
            publish_delay_sec=max(0.0, float(args.lidar_publish_delay_sec)),
        )
        self.gnss_fault = ChannelFault(
            mode=args.gnss_mode,
            drop_every=max(1, int(args.gnss_drop_every)),
            stamp_offset_sec=float(args.gnss_stamp_offset_sec),
            publish_delay_sec=max(0.0, float(args.gnss_publish_delay_sec)),
        )

        self.camera_pub = self.create_publisher(Image, args.camera_output_topic, 10)
        self.lidar_pub = self.create_publisher(PointCloud2, args.lidar_output_topic, 10)
        self.gnss_pub = self.create_publisher(Gnssins64, args.gnss_output_topic, 10)

        self.create_subscription(Image, args.camera_input_topic, self.cb_camera, 10)
        self.create_subscription(PointCloud2, args.lidar_input_topic, self.cb_lidar, 10)
        self.create_subscription(Gnssins64, args.gnss_input_topic, self.cb_gnss, 10)
        self.create_timer(0.01, self.flush_pending)

        self.pending: List[tuple[float, Any, Any]] = []
        self.counts: Dict[str, Dict[str, int]] = {
            "camera": {"received": 0, "published": 0, "dropped": 0},
            "lidar": {"received": 0, "published": 0, "dropped": 0},
            "gnss": {"received": 0, "published": 0, "dropped": 0},
        }

        self.get_logger().info(
            "Runtime fault injector ready. "
            f"camera={self.camera_fault.mode}, lidar={self.lidar_fault.mode}, gnss={self.gnss_fault.mode}, "
            f"start={self.fault_start_sec:.2f}s, duration={self.fault_duration_sec:.2f}s"
        )

    def fault_active(self) -> bool:
        elapsed = time.monotonic() - self.start_wall
        if elapsed < self.fault_start_sec:
            return False
        if self.fault_duration_sec < 0.0:
            return True
        return elapsed <= self.fault_start_sec + self.fault_duration_sec

    def cb_camera(self, msg: Image) -> None:
        self.counts["camera"]["received"] += 1
        processed = self.apply_camera_fault(msg, self.camera_fault, self.counts["camera"]["received"])
        if processed is None:
            self.counts["camera"]["dropped"] += 1
            return
        self.publish_or_queue(self.camera_pub, processed, self.camera_fault.publish_delay_sec, "camera")

    def cb_lidar(self, msg: PointCloud2) -> None:
        self.counts["lidar"]["received"] += 1
        processed = self.apply_generic_fault(msg, self.lidar_fault, self.counts["lidar"]["received"])
        if processed is None:
            self.counts["lidar"]["dropped"] += 1
            return
        self.publish_or_queue(self.lidar_pub, processed, self.lidar_fault.publish_delay_sec, "lidar")

    def cb_gnss(self, msg: Gnssins64) -> None:
        self.counts["gnss"]["received"] += 1
        processed = self.apply_generic_fault(msg, self.gnss_fault, self.counts["gnss"]["received"])
        if processed is None:
            self.counts["gnss"]["dropped"] += 1
            return
        self.publish_or_queue(self.gnss_pub, processed, self.gnss_fault.publish_delay_sec, "gnss")

    def should_drop(self, fault: ChannelFault, index: int) -> bool:
        if not self.fault_active():
            return False
        return fault.mode == "drop" and fault.drop_every > 0 and index % fault.drop_every == 0

    def apply_generic_fault(self, msg: Any, fault: ChannelFault, index: int) -> Optional[Any]:
        if self.should_drop(fault, index):
            return None
        out = copy.deepcopy(msg)
        if self.fault_active() and abs(fault.stamp_offset_sec) > 1e-6 and hasattr(out, "header"):
            shift_stamp(out.header.stamp, fault.stamp_offset_sec)
        return out

    def apply_camera_fault(self, msg: Image, fault: ChannelFault, index: int) -> Optional[Image]:
        if self.should_drop(fault, index):
            return None

        out = copy.deepcopy(msg)
        if self.fault_active() and fault.mode in {"blur", "blank"}:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if fault.mode == "blur":
                cv_image = cv2.GaussianBlur(cv_image, (fault.blur_kernel, fault.blur_kernel), 0)
            elif fault.mode == "blank":
                cv_image[:] = 0
            out = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            out.header = copy.deepcopy(msg.header)

        if self.fault_active() and abs(fault.stamp_offset_sec) > 1e-6:
            shift_stamp(out.header.stamp, fault.stamp_offset_sec)
        return out

    def publish_or_queue(self, publisher: Any, msg: Any, delay_sec: float, key: str) -> None:
        if self.fault_active() and delay_sec > 1e-6:
            self.pending.append((time.monotonic() + delay_sec, publisher, msg))
            return
        publisher.publish(msg)
        self.counts[key]["published"] += 1

    def flush_pending(self) -> None:
        if not self.pending:
            return
        now = time.monotonic()
        ready = [item for item in self.pending if item[0] <= now]
        self.pending = [item for item in self.pending if item[0] > now]
        for _, publisher, msg in ready:
            publisher.publish(msg)
            if isinstance(msg, Image):
                self.counts["camera"]["published"] += 1
            elif isinstance(msg, PointCloud2):
                self.counts["lidar"]["published"] += 1
            elif isinstance(msg, Gnssins64):
                self.counts["gnss"]["published"] += 1

    def shutdown(self) -> None:
        while self.pending:
            _, publisher, msg = self.pending.pop(0)
            publisher.publish(msg)
            if isinstance(msg, Image):
                self.counts["camera"]["published"] += 1
            elif isinstance(msg, PointCloud2):
                self.counts["lidar"]["published"] += 1
            elif isinstance(msg, Gnssins64):
                self.counts["gnss"]["published"] += 1

        if self.log_path is not None:
            payload = {
                "fault_start_sec": self.fault_start_sec,
                "fault_duration_sec": self.fault_duration_sec,
                "counts": self.counts,
                "camera_fault": self.camera_fault.__dict__,
                "lidar_fault": self.lidar_fault.__dict__,
                "gnss_fault": self.gnss_fault.__dict__,
            }
            self.log_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-input-topic", default="/camera1/image_raw")
    parser.add_argument("--camera-output-topic", default="/fault_injected/camera1/image_raw")
    parser.add_argument("--lidar-input-topic", default="/lidar_points")
    parser.add_argument("--lidar-output-topic", default="/fault_injected/lidar_points")
    parser.add_argument("--gnss-input-topic", default="/gongji_gnss_ins_64")
    parser.add_argument("--gnss-output-topic", default="/fault_injected/gongji_gnss_ins_64")
    parser.add_argument("--camera-mode", choices=("none", "blur", "blank", "drop"), default="none")
    parser.add_argument("--lidar-mode", choices=("none", "drop"), default="none")
    parser.add_argument("--gnss-mode", choices=("none", "drop"), default="none")
    parser.add_argument("--camera-drop-every", type=int, default=2)
    parser.add_argument("--lidar-drop-every", type=int, default=2)
    parser.add_argument("--gnss-drop-every", type=int, default=2)
    parser.add_argument("--camera-blur-kernel", type=int, default=19)
    parser.add_argument("--camera-stamp-offset-sec", type=float, default=0.0)
    parser.add_argument("--lidar-stamp-offset-sec", type=float, default=0.0)
    parser.add_argument("--gnss-stamp-offset-sec", type=float, default=0.0)
    parser.add_argument("--camera-publish-delay-sec", type=float, default=0.0)
    parser.add_argument("--lidar-publish-delay-sec", type=float, default=0.0)
    parser.add_argument("--gnss-publish-delay-sec", type=float, default=0.0)
    parser.add_argument("--fault-start-sec", type=float, default=0.0)
    parser.add_argument("--fault-duration-sec", type=float, default=-1.0)
    parser.add_argument("--log-path", default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = RuntimeFaultInjector(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
