#!/usr/bin/env python3
import argparse
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray

from cone_interfaces.msg import ConeArray
from drd25_msgs.msg import Map
from gnss_ins_msg.msg import Gnssins64
from test_cone_segmentation.msg import ThreeDConeArray


class FullDemoRecorder(Node):
    def __init__(self, output_path: Path, fps: float, idle_timeout: float, startup_timeout: float) -> None:
        super().__init__("drd26_full_demo_recorder")
        self.output_path = output_path
        self.idle_timeout = idle_timeout
        self.startup_timeout = startup_timeout
        self.bridge = CvBridge()

        self.start_time = time.time()
        self.last_activity = self.start_time
        self.received_any = False
        self.frame_index = 0

        self.latest_debug_image = None
        self.latest_yolo = None
        self.latest_lidar = None
        self.latest_fusion = None
        self.latest_global_markers = []
        self.latest_vehicle_odom = None
        self.latest_gnss = None

        self.vehicle_path = deque(maxlen=4000)
        self.global_cone_history = deque(maxlen=8000)

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.writer = cv2.VideoWriter(str(output_path), fourcc, fps, (1600, 900))
        if not self.writer.isOpened():
            raise RuntimeError(f"Failed to open output video: {output_path}")

        self.create_subscription(Image, "/yolo/debug_image", self.cb_debug_image, 10)
        self.create_subscription(ConeArray, "/yolo/cones", self.cb_yolo, 10)
        self.create_subscription(ThreeDConeArray, "/cone_detection_custom", self.cb_lidar, 10)
        self.create_subscription(Map, "/perception/fusion/map", self.cb_fusion, 10)
        self.create_subscription(MarkerArray, "/global_map", self.cb_global_map, 10)
        self.create_subscription(Odometry, "/vehicle_odom", self.cb_vehicle_odom, 10)
        self.create_subscription(Gnssins64, "/gongji_gnss_ins_64", self.cb_gnss, 10)

        self.create_timer(1.0 / fps, self.write_frame)

    def touch(self) -> None:
        self.received_any = True
        self.last_activity = time.time()

    def cb_debug_image(self, msg: Image) -> None:
        self.latest_debug_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.touch()

    def cb_yolo(self, msg: ConeArray) -> None:
        self.latest_yolo = msg
        self.touch()

    def cb_lidar(self, msg: ThreeDConeArray) -> None:
        self.latest_lidar = msg
        self.touch()

    def cb_fusion(self, msg: Map) -> None:
        self.latest_fusion = msg
        self.touch()

    def cb_global_map(self, msg: MarkerArray) -> None:
        markers = []
        for marker in msg.markers:
            if marker.action == Marker.DELETEALL:
                continue
            x = float(marker.pose.position.x)
            y = float(marker.pose.position.y)
            color = self.marker_color(marker)
            markers.append((x, y, int(marker.id), color))
            self.global_cone_history.append((x, y, color))
        self.latest_global_markers = markers
        self.touch()

    def cb_vehicle_odom(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self.latest_vehicle_odom = (x, y)
        self.vehicle_path.append((x, y))
        self.touch()

    def cb_gnss(self, msg: Gnssins64) -> None:
        self.latest_gnss = msg
        self.touch()

    def finished(self) -> bool:
        now = time.time()
        if not self.received_any:
            return now - self.start_time > self.startup_timeout
        return now - self.last_activity > self.idle_timeout

    def write_frame(self) -> None:
        if not self.received_any:
            return

        canvas = np.full((900, 1600, 3), 246, dtype=np.uint8)
        elapsed = time.time() - self.start_time

        self.draw_header(canvas, elapsed)
        self.draw_debug_panel(canvas[70:530, 20:820])
        self.draw_global_panel(canvas[70:530, 840:1580])
        self.draw_local_panel(canvas[560:880, 20:820])
        self.draw_status_panel(canvas[560:880, 840:1580], elapsed)

        self.writer.write(canvas)
        self.frame_index += 1

    def draw_header(self, canvas: np.ndarray, elapsed: float) -> None:
        cv2.putText(canvas, "DRd26 Full SLAM Demo", (20, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (35, 35, 35), 2, cv2.LINE_AA)
        cv2.putText(
            canvas,
            f"elapsed {elapsed:6.1f}s   frame {self.frame_index:5d}   recording follows bag until topics go idle",
            (20, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (75, 75, 75),
            2,
            cv2.LINE_AA,
        )

    def draw_debug_panel(self, panel: np.ndarray) -> None:
        panel[:] = 255
        cv2.putText(panel, "YOLO debug image", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (80, 80, 80), 2, cv2.LINE_AA)
        if self.latest_debug_image is None:
            cv2.putText(panel, "waiting for /yolo/debug_image", (55, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (120, 120, 120), 2, cv2.LINE_AA)
            return

        image = self.fit_into(self.latest_debug_image, panel.shape[1], panel.shape[0] - 32)
        panel[32:32 + image.shape[0], 0:image.shape[1]] = image

    def draw_global_panel(self, panel: np.ndarray) -> None:
        panel[:] = 255
        cv2.putText(panel, "Global map view (camera follows vehicle)", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (80, 80, 80), 2, cv2.LINE_AA)

        h, w = panel.shape[:2]
        center_px = (w // 2, h // 2 + 18)
        scale = 42.0
        vehicle_xy = self.latest_vehicle_odom if self.latest_vehicle_odom is not None else (0.0, 0.0)

        for x in range(0, w, 42):
            cv2.line(panel, (x, 32), (x, h), (235, 235, 235), 1)
        for y in range(32, h, 42):
            cv2.line(panel, (0, y), (w, y), (235, 235, 235), 1)
        cv2.line(panel, (center_px[0], 32), (center_px[0], h), (180, 180, 180), 1)
        cv2.line(panel, (0, center_px[1]), (w, center_px[1]), (180, 180, 180), 1)

        for idx in range(1, len(self.vehicle_path)):
            p1 = self.global_to_px(self.vehicle_path[idx - 1], vehicle_xy, center_px, scale)
            p2 = self.global_to_px(self.vehicle_path[idx], vehicle_xy, center_px, scale)
            cv2.line(panel, p1, p2, (185, 105, 55), 2)

        for x, y, color in self.global_cone_history:
            px = self.global_to_px((x, y), vehicle_xy, center_px, scale)
            if self.in_panel(px, panel.shape):
                cv2.circle(panel, px, 3, self.fade_color(color, keep=0.68), -1)

        for x, y, marker_id, color in self.latest_global_markers:
            px = self.global_to_px((x, y), vehicle_xy, center_px, scale)
            if self.in_panel(px, panel.shape):
                cv2.circle(panel, px, 8, color, -1)
                cv2.circle(panel, px, 10, (255, 255, 255), 1)
                cv2.putText(panel, f"id {marker_id}", (px[0] + 10, px[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (65, 65, 65), 1, cv2.LINE_AA)

        cv2.circle(panel, center_px, 9, (25, 25, 25), -1)
        cv2.putText(panel, "vehicle", (center_px[0] + 10, center_px[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (25, 25, 25), 1, cv2.LINE_AA)
        cv2.putText(panel, "x ->", (w - 50, center_px[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 120, 120), 1, cv2.LINE_AA)
        cv2.putText(panel, "y", (center_px[0] + 8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 120, 120), 1, cv2.LINE_AA)

    def draw_local_panel(self, panel: np.ndarray) -> None:
        panel[:] = 255
        cv2.putText(panel, "Local detections in LiDAR frame", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (80, 80, 80), 2, cv2.LINE_AA)

        h, w = panel.shape[:2]
        origin = (120, h // 2 + 10)
        scale = 28.0

        for r in range(1, 8):
            cv2.circle(panel, origin, int(r * 2.0 * scale), (238, 238, 238), 1)
        cv2.line(panel, (origin[0], 32), (origin[0], h - 10), (180, 180, 180), 1)
        cv2.line(panel, (origin[0], origin[1]), (w - 20, origin[1]), (180, 180, 180), 1)
        cv2.arrowedLine(panel, origin, (origin[0] + 90, origin[1]), (90, 90, 90), 2)
        cv2.arrowedLine(panel, origin, (origin[0], origin[1] - 90), (90, 90, 90), 2)
        cv2.putText(panel, "x forward", (origin[0] + 96, origin[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (95, 95, 95), 1, cv2.LINE_AA)
        cv2.putText(panel, "y left", (origin[0] + 10, origin[1] - 96), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (95, 95, 95), 1, cv2.LINE_AA)

        if self.latest_lidar is not None:
            for cone in self.latest_lidar.cones:
                px = self.local_to_px(float(cone.center.x), float(cone.center.y), origin, scale)
                if self.in_panel(px, panel.shape):
                    cv2.circle(panel, px, 5, (220, 80, 80), -1)

        if self.latest_fusion is not None:
            for cone in self.latest_fusion.track:
                px = self.local_to_px(float(cone.x), float(cone.y), origin, scale)
                if self.in_panel(px, panel.shape):
                    cv2.rectangle(panel, (px[0] - 5, px[1] - 5), (px[0] + 5, px[1] + 5), self.cone_color(int(cone.color)), -1)

        legend_y = 46
        cv2.circle(panel, (560, legend_y), 5, (220, 80, 80), -1)
        cv2.putText(panel, "LiDAR clusters", (575, legend_y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (70, 70, 70), 1, cv2.LINE_AA)
        cv2.rectangle(panel, (555, legend_y + 28), (565, legend_y + 38), (0, 200, 255), -1)
        cv2.putText(panel, "Fused cones", (575, legend_y + 37), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (70, 70, 70), 1, cv2.LINE_AA)

    def draw_status_panel(self, panel: np.ndarray, elapsed: float) -> None:
        panel[:] = 255
        cv2.putText(panel, "Status dashboard", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (80, 80, 80), 2, cv2.LINE_AA)

        yolo_n = len(self.latest_yolo.cones) if self.latest_yolo is not None else 0
        lidar_n = len(self.latest_lidar.cones) if self.latest_lidar is not None else 0
        fused_n = len(self.latest_fusion.track) if self.latest_fusion is not None else 0
        global_n = len(self.latest_global_markers)

        lines = [
            f"YOLO 2D detections   : {yolo_n}",
            f"LiDAR 3D detections  : {lidar_n}",
            f"Fusion map cones     : {fused_n}",
            f"Global map markers   : {global_n}",
            f"Elapsed              : {elapsed:6.1f} s",
            f"Video file            : {self.output_path.name}",
            "",
        ]

        if self.latest_vehicle_odom is not None:
            lines.append(f"Vehicle odom x/y     : {self.latest_vehicle_odom[0]:8.3f}, {self.latest_vehicle_odom[1]:8.3f}")

        if self.latest_gnss is not None:
            lines.append(f"GNSS lat/lon         : {float(self.latest_gnss.latitude):.8f}, {float(self.latest_gnss.longitude):.8f}")
            lines.append(f"GNSS EN speed        : {float(getattr(self.latest_gnss, 'vel_e', 0.0)):6.3f}, {float(getattr(self.latest_gnss, 'vel_n', 0.0)):6.3f}")
            lines.append(f"Yaw / gyro_z         : {float(getattr(self.latest_gnss, 'yaw', 0.0)):7.3f}, {float(getattr(self.latest_gnss, 'imu_gyro_z', 0.0)):7.3f}")

        lines.extend(
            [
                "",
                "Pipeline:",
                "image_raw -> YOLO -> LiDAR segmentation -> fusion -> SLAM global_map",
                "The recorder stops only after the bag finishes and all observed topics go idle.",
            ]
        )

        y = 58
        for line in lines:
            cv2.putText(panel, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (55, 55, 55), 1, cv2.LINE_AA)
            y += 28

    def fit_into(self, image: np.ndarray, target_w: int, target_h: int) -> np.ndarray:
        h, w = image.shape[:2]
        scale = min(target_w / max(w, 1), target_h / max(h, 1))
        new_w = max(1, int(w * scale))
        new_h = max(1, int(h * scale))
        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
        panel = np.full((target_h, target_w, 3), 255, dtype=np.uint8)
        x0 = (target_w - new_w) // 2
        y0 = (target_h - new_h) // 2
        panel[y0:y0 + new_h, x0:x0 + new_w] = resized
        return panel

    def global_to_px(self, point, vehicle_center, panel_center, scale):
        x, y = point
        cx, cy = vehicle_center
        return (int(panel_center[0] + (x - cx) * scale), int(panel_center[1] - (y - cy) * scale))

    def local_to_px(self, x: float, y: float, origin, scale):
        return (int(origin[0] + x * scale), int(origin[1] - y * scale))

    def in_panel(self, point, shape) -> bool:
        x, y = point
        h, w = shape[:2]
        return 0 <= x < w and 32 <= y < h

    def cone_color(self, color_id: int):
        if color_id == 0:
            return (255, 90, 60)
        if color_id == 1:
            return (60, 90, 255)
        if color_id in (2, 3):
            return (0, 220, 255)
        return (160, 160, 160)

    def marker_color(self, marker: Marker):
        r = int(np.clip(float(marker.color.r) * 255.0, 0.0, 255.0))
        g = int(np.clip(float(marker.color.g) * 255.0, 0.0, 255.0))
        b = int(np.clip(float(marker.color.b) * 255.0, 0.0, 255.0))
        return (b, g, r)

    def fade_color(self, color, keep: float = 0.65):
        keep = float(np.clip(keep, 0.0, 1.0))
        white = 255.0 * (1.0 - keep)
        b, g, r = color
        return (
            int(np.clip(b * keep + white, 0.0, 255.0)),
            int(np.clip(g * keep + white, 0.0, 255.0)),
            int(np.clip(r * keep + white, 0.0, 255.0)),
        )

    def close(self) -> None:
        self.writer.release()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--idle-timeout", type=float, default=12.0)
    parser.add_argument("--startup-timeout", type=float, default=60.0)
    args = parser.parse_args()

    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = FullDemoRecorder(output_path, args.fps, args.idle_timeout, args.startup_timeout)
    try:
        while rclpy.ok() and not node.finished():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
