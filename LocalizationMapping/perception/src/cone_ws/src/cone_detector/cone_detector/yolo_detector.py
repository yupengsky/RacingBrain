import os
import json
import time
from configparser import ConfigParser
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

from cone_interfaces.msg import Cone
from cone_interfaces.msg import ConeArray


def find_path_config() -> Optional[Path]:
    env_config = os.environ.get("DRD26_PATH_CONFIG")
    if env_config and Path(env_config).exists():
        return Path(env_config)

    for parent in Path(__file__).resolve().parents:
        candidate = parent / "config" / "hardcoded_paths.ini"
        if candidate.exists():
            return candidate

    candidate = Path.cwd() / "config" / "hardcoded_paths.ini"
    if candidate.exists():
        return candidate

    return None


def load_default_model_path() -> str:
    config_path = find_path_config()
    if config_path is None:
        return ""

    parser = ConfigParser()
    parser.read(config_path, encoding="utf-8")
    if not parser.has_option("models", "yolo_runtime"):
        return ""
    return parser.get("models", "yolo_runtime")


class YoloDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detector")

        self.bridge = CvBridge()
        self.last_inference_time = 0.0
        self.hsv_mismatch_count = 0
        self.hsv_checked_count = 0

        self.declare_parameter("model_path", load_default_model_path())
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("max_fps", 10.0)
        self.declare_parameter("enable_hsv_check", False)
        self.declare_parameter("evaluation.enable_debug_metrics", False)
        self.declare_parameter("runtime_health.enable_metrics", False)

        self.model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.max_fps = float(self.get_parameter("max_fps").value)
        self.enable_hsv_check = bool(self.get_parameter("enable_hsv_check").value)
        self.enable_debug_metrics = bool(self.get_parameter("evaluation.enable_debug_metrics").value)
        self.enable_health_metrics = bool(self.get_parameter("runtime_health.enable_metrics").value)
        self.metrics_enabled = self.enable_debug_metrics or self.enable_health_metrics
        self.min_period = 1.0 / self.max_fps if self.max_fps > 0.0 else 0.0

        if not self.model_path:
            raise RuntimeError("Parameter 'model_path' is empty.")
        if not Path(self.model_path).exists():
            raise FileNotFoundError(f"YOLO model not found: {self.model_path}")

        self.get_logger().info(f"Loading YOLO model from: {self.model_path}")
        self.model = YOLO(self.model_path)

        self.debug_pub = self.create_publisher(Image, "/yolo/debug_image", 10)
        self.cones_pub = self.create_publisher(ConeArray, "/yolo/cones", 10)
        self.metrics_pub = None
        if self.metrics_enabled:
            self.metrics_pub = self.create_publisher(String, "/perception/yolo/evaluation/metrics", 10)
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_only_callback,
            10,
        )

        self.get_logger().info(
            "YOLO detector ready. "
            f"topic={self.image_topic}, conf_threshold={self.conf_threshold}, max_fps={self.max_fps}"
        )

    def image_only_callback(self, image_msg: Image) -> None:
        callback_start = time.perf_counter()
        now = time.monotonic()
        if self.min_period > 0.0 and now - self.last_inference_time < self.min_period:
            self._publish_metrics(
                image_msg,
                {
                    "event": "skipped_throttle",
                    "skipped": True,
                    "total_ms": self._elapsed_ms(callback_start),
                    "max_fps": self.max_fps,
                },
            )
            return
        self.last_inference_time = now

        try:
            convert_start = time.perf_counter()
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            convert_ms = self._elapsed_ms(convert_start)
        except Exception as exc:
            self.get_logger().error(f"Failed to convert image: {exc}")
            self._publish_metrics(
                image_msg,
                {
                    "event": "conversion_failed",
                    "skipped": True,
                    "total_ms": self._elapsed_ms(callback_start),
                },
            )
            return

        try:
            inference_start = time.perf_counter()
            result = self.model.predict(
                source=cv_image,
                conf=self.conf_threshold,
                verbose=False,
            )[0]
            inference_ms = self._elapsed_ms(inference_start)
        except Exception as exc:
            self.get_logger().error(f"YOLO inference failed: {exc}")
            self._publish_metrics(
                image_msg,
                {
                    "event": "inference_failed",
                    "skipped": True,
                    "convert_ms": convert_ms,
                    "total_ms": self._elapsed_ms(callback_start),
                },
            )
            return

        build_start = time.perf_counter()
        cone_array = self._build_cone_array(image_msg, result)
        build_ms = self._elapsed_ms(build_start)

        draw_start = time.perf_counter()
        debug_image = self.draw_detections(cv_image.copy(), result)
        draw_ms = self._elapsed_ms(draw_start)

        publish_start = time.perf_counter()
        self.cones_pub.publish(cone_array)
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_msg.header = image_msg.header
            self.debug_pub.publish(debug_msg)
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish debug image: {exc}")
        publish_ms = self._elapsed_ms(publish_start)

        self._publish_metrics(
            image_msg,
            {
                "event": "processed",
                "skipped": False,
                "convert_ms": convert_ms,
                "inference_ms": inference_ms,
                "build_msg_ms": build_ms,
                "draw_ms": draw_ms,
                "publish_ms": publish_ms,
                "total_ms": self._elapsed_ms(callback_start),
                "cone_count": len(cone_array.cones),
                "image_width": image_msg.width,
                "image_height": image_msg.height,
                "conf_threshold": self.conf_threshold,
                "max_fps": self.max_fps,
            },
        )

    def draw_detections(self, image: np.ndarray, result) -> np.ndarray:
        boxes = getattr(result, "boxes", None)
        if boxes is None or boxes.xyxy is None:
            return image

        xyxy = boxes.xyxy.cpu().numpy()
        confidences = boxes.conf.cpu().numpy()
        class_ids = boxes.cls.cpu().numpy().astype(int)

        for bbox, confidence, class_id in zip(xyxy, confidences, class_ids):
            x1, y1, x2, y2 = [int(v) for v in bbox]
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(image.shape[1] - 1, x2)
            y2 = min(image.shape[0] - 1, y2)

            class_name = self._get_class_name(class_id)
            mapped_color = self._map_color(class_name)
            box_color = self._color_bgr(mapped_color)
            hsv_note = ""

            if self.enable_hsv_check and x2 > x1 and y2 > y1:
                observed_color = self._classify_hsv_color(image[y1:y2, x1:x2])
                expected_color = self._expected_color(mapped_color)
                if expected_color is not None and observed_color is not None:
                    self.hsv_checked_count += 1
                    if observed_color != expected_color:
                        self.hsv_mismatch_count += 1
                        box_color = (0, 0, 255)
                        hsv_note = f" hsv:{observed_color}"

            label = f"{class_name} {confidence:.2f}{hsv_note}"
            cv2.rectangle(image, (x1, y1), (x2, y2), box_color, 2)
            cv2.putText(
                image,
                label,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                box_color,
                1,
                cv2.LINE_AA,
            )

        return image

    def _build_cone_array(self, image_msg: Image, result) -> ConeArray:
        cone_array = ConeArray()
        cone_array.header = image_msg.header

        boxes = getattr(result, "boxes", None)
        if boxes is None or boxes.xyxy is None:
            return cone_array

        xyxy = boxes.xyxy.cpu().numpy()
        confidences = boxes.conf.cpu().numpy()
        class_ids = boxes.cls.cpu().numpy().astype(int)

        for bbox, confidence, class_id in zip(xyxy, confidences, class_ids):
            x1, y1, x2, y2 = [float(v) for v in bbox]
            cone = Cone()
            cone.center.x = (x1 + x2) * 0.5
            cone.center.y = (y1 + y2) * 0.5
            cone.size.x = max(0.0, x2 - x1)
            cone.size.y = max(0.0, y2 - y1)
            cone.confidence = float(confidence)
            cone.color = self._map_color(self._get_class_name(class_id))
            cone_array.cones.append(cone)

        return cone_array

    def _get_class_name(self, class_id: int) -> str:
        names = getattr(self.model, "names", {})
        if isinstance(names, dict):
            return str(names.get(class_id, class_id))
        if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
            return str(names[class_id])
        return str(class_id)

    def _map_color(self, class_name: str) -> int:
        key = class_name.strip().lower()
        mapping = {
            "blue": Cone.BLUE,
            "blue_cone": Cone.BLUE,
            "red": Cone.RED,
            "red_cone": Cone.RED,
            "yellow_big": Cone.YELLOW_BIG,
            "big_yellow_cone": Cone.YELLOW_BIG,
            "yellow_small": Cone.YELLOW_SMALL,
            "small_yellow_cone": Cone.YELLOW_SMALL,
            "orange_big": Cone.YELLOW_BIG,
            "orange_small": Cone.YELLOW_SMALL,
            "big_orange": Cone.YELLOW_BIG,
            "small_orange": Cone.YELLOW_SMALL,
            "unknown": Cone.UNKNOWN,
        }
        return mapping.get(key, Cone.UNKNOWN)

    def _expected_color(self, cone_color: int) -> Optional[str]:
        if cone_color == Cone.BLUE:
            return "blue"
        if cone_color == Cone.RED:
            return "red"
        if cone_color in (Cone.YELLOW_BIG, Cone.YELLOW_SMALL):
            return "yellow"
        return None

    def _classify_hsv_color(self, roi: np.ndarray) -> Optional[str]:
        if roi.size == 0:
            return None

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]
        mask = (saturation > 50) & (value > 50)
        if not np.any(mask):
            return None

        hue = hsv[:, :, 0][mask]
        median_hue = float(np.median(hue))
        if median_hue < 15.0 or median_hue >= 160.0:
            return "red"
        if 15.0 <= median_hue < 45.0:
            return "yellow"
        if 90.0 <= median_hue < 140.0:
            return "blue"
        return None

    def _color_bgr(self, cone_color: int) -> tuple[int, int, int]:
        if cone_color == Cone.BLUE:
            return (255, 0, 0)
        if cone_color == Cone.RED:
            return (0, 0, 255)
        if cone_color in (Cone.YELLOW_BIG, Cone.YELLOW_SMALL):
            return (0, 255, 255)
        return (180, 180, 180)

    def _publish_metrics(self, image_msg: Image, payload: dict) -> None:
        if self.metrics_pub is None:
            return
        msg = String()
        payload = {
            "component": "yolo",
            "stamp": self._stamp_sec(image_msg),
            "frame_id": image_msg.header.frame_id,
            "hsv_checked_count": self.hsv_checked_count,
            "hsv_mismatch_count": self.hsv_mismatch_count,
            **payload,
        }
        msg.data = json.dumps(payload, sort_keys=True)
        self.metrics_pub.publish(msg)

    @staticmethod
    def _stamp_sec(image_msg: Image) -> float:
        return float(image_msg.header.stamp.sec) + float(image_msg.header.stamp.nanosec) * 1e-9

    @staticmethod
    def _elapsed_ms(start: float) -> float:
        return (time.perf_counter() - start) * 1000.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetectorNode()
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
