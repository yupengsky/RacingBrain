#!/usr/bin/env python3
"""Benchmark learned YOLO detection against CPU classical vision baselines."""

from __future__ import annotations

import argparse
import csv
import json
import os
import time
from configparser import ConfigParser
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image


def find_workspace() -> Path:
    return Path(__file__).resolve().parents[2]


def load_config_path(workspace: Path, explicit: Optional[str]) -> Path:
    if explicit:
        return Path(explicit)
    env_path = os.environ.get("DRD26_PATH_CONFIG")
    if env_path:
        return Path(env_path)
    return workspace / "LocalizationMapping" / "config" / "hardcoded_paths.ini"


def config_value(config_path: Path, section: str, option: str, fallback: str = "") -> str:
    parser = ConfigParser()
    parser.read(config_path, encoding="utf-8")
    if parser.has_option(section, option):
        return parser.get(section, option)
    return fallback


def stamp_sec(msg: Image) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    if msg.encoding not in {"rgb8", "bgr8", "mono8"}:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")
    channels = 1 if msg.encoding == "mono8" else 3
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    image = raw.reshape((msg.height, msg.step // channels, channels))[:, : msg.width, :]
    if msg.encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if msg.encoding == "mono8":
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return image.copy()


def iter_images(bag_path: Path, topic: str, max_frames: int, stride: int) -> Iterable[Image]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise RuntimeError(f"Topic not found in bag: {topic}")
    msg_type = get_message(topic_types[topic])

    seen = 0
    yielded = 0
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name != topic:
            continue
        if seen % stride == 0:
            yield deserialize_message(data, msg_type)
            yielded += 1
            if max_frames > 0 and yielded >= max_frames:
                return
        seen += 1


def build_color_masks(image_bgr: np.ndarray) -> Dict[str, np.ndarray]:
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    return {
        "red": cv2.inRange(hsv, (0, 60, 45), (10, 255, 255))
        | cv2.inRange(hsv, (170, 60, 45), (180, 255, 255)),
        "yellow": cv2.inRange(hsv, (14, 55, 50), (42, 255, 255)),
        "blue": cv2.inRange(hsv, (85, 50, 40), (135, 255, 255)),
    }


def detect_hsv_components(image_bgr: np.ndarray) -> List[Dict[str, Any]]:
    masks = build_color_masks(image_bgr)
    kernel = np.ones((3, 3), dtype=np.uint8)
    detections: List[Dict[str, Any]] = []
    image_area = float(image_bgr.shape[0] * image_bgr.shape[1])

    for color, mask in masks.items():
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < 18.0 or area > image_area * 0.08:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w < 3 or h < 5:
                continue
            aspect = float(w) / float(h)
            if aspect > 2.5 or aspect < 0.15:
                continue
            detections.append({"color": color, "x": x, "y": y, "w": w, "h": h, "area": area})
    return detections


def row_occupancy_width(mask_roi: np.ndarray, y0: int, y1: int) -> float:
    if y1 <= y0:
        return 0.0
    band = mask_roi[y0:y1, :]
    cols = np.where(np.any(band > 0, axis=0))[0]
    if cols.size == 0:
        return 0.0
    return float(cols[-1] - cols[0] + 1)


def box_iou(a: Dict[str, Any], b: Dict[str, Any]) -> float:
    ax1, ay1 = float(a["x"]), float(a["y"])
    ax2, ay2 = ax1 + float(a["w"]), ay1 + float(a["h"])
    bx1, by1 = float(b["x"]), float(b["y"])
    bx2, by2 = bx1 + float(b["w"]), by1 + float(b["h"])
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    union = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return 0.0 if union <= 0.0 else inter / union


def non_max_suppress(candidates: List[Dict[str, Any]], iou_threshold: float = 0.35) -> List[Dict[str, Any]]:
    kept: List[Dict[str, Any]] = []
    for candidate in sorted(candidates, key=lambda item: float(item.get("score", 0.0)), reverse=True):
        if all(box_iou(candidate, old) < iou_threshold for old in kept):
            kept.append(candidate)
    return kept


def detect_classical_cones(image_bgr: np.ndarray) -> List[Dict[str, Any]]:
    """Color mask + connected components + cone-like shape tests.

    This is still a classical CPU baseline, not a learned detector. It mirrors
    the LiDAR cluster path more closely than the raw HSV count: generate
    candidate components, reject implausible geometry, then merge overlaps.
    """
    height, width = image_bgr.shape[:2]
    image_area = float(height * width)
    roi_top = int(round(height * 0.18))
    masks = build_color_masks(image_bgr)
    open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    candidates: List[Dict[str, Any]] = []

    for color, raw_mask in masks.items():
        mask = np.zeros_like(raw_mask)
        mask[roi_top:, :] = raw_mask[roi_top:, :]
        mask = cv2.medianBlur(mask, 3)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < 24.0 or area > image_area * 0.045:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if h < 8 or w < 4 or h > height * 0.75:
                continue
            aspect = float(w) / float(h)
            if aspect < 0.16 or aspect > 1.65:
                continue

            rect_area = float(w * h)
            fill_ratio = area / rect_area if rect_area > 0.0 else 0.0
            if fill_ratio < 0.08 or fill_ratio > 0.86:
                continue

            hull = cv2.convexHull(contour)
            hull_area = float(cv2.contourArea(hull))
            solidity = area / hull_area if hull_area > 1.0 else 0.0
            if solidity < 0.30:
                continue

            roi = mask[y : y + h, x : x + w]
            top_width = row_occupancy_width(roi, 0, max(1, int(0.35 * h)))
            mid_width = row_occupancy_width(roi, int(0.30 * h), max(int(0.70 * h), int(0.30 * h) + 1))
            bottom_width = row_occupancy_width(roi, int(0.62 * h), h)
            if h >= 18 and bottom_width < max(3.0, top_width * 0.85):
                continue
            if mid_width <= 0.0:
                continue

            moments = cv2.moments(contour)
            if moments["m00"] <= 1e-6:
                continue
            centroid_y = float(moments["m01"] / moments["m00"])
            normalized_centroid_y = (centroid_y - float(y)) / float(h)
            if normalized_centroid_y < 0.35 or normalized_centroid_y > 0.82:
                continue

            score = (
                min(1.0, area / 220.0)
                + min(1.0, solidity)
                + min(1.0, fill_ratio * 2.0)
                + min(1.0, bottom_width / max(1.0, mid_width))
            )
            candidates.append(
                {
                    "color": color,
                    "x": int(x),
                    "y": int(y),
                    "w": int(w),
                    "h": int(h),
                    "area": area,
                    "fill_ratio": fill_ratio,
                    "solidity": solidity,
                    "score": score,
                }
            )

    return non_max_suppress(candidates)


@dataclass
class Stat:
    count: int
    mean: Optional[float]
    median: Optional[float]
    p95: Optional[float]
    minimum: Optional[float]
    maximum: Optional[float]


def summarize(values: List[float]) -> Stat:
    if not values:
        return Stat(0, None, None, None, None, None)
    arr = np.asarray(values, dtype=np.float64)
    return Stat(
        count=int(arr.size),
        mean=float(np.mean(arr)),
        median=float(np.median(arr)),
        p95=float(np.percentile(arr, 95)),
        minimum=float(np.min(arr)),
        maximum=float(np.max(arr)),
    )


def stat_dict(values: List[float]) -> Dict[str, Any]:
    stat = summarize(values)
    return {
        "count": stat.count,
        "mean": stat.mean,
        "median": stat.median,
        "p95": stat.p95,
        "min": stat.minimum,
        "max": stat.maximum,
    }


def main() -> int:
    workspace = find_workspace()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=None)
    parser.add_argument("--bag", default=None)
    parser.add_argument("--model", default=None)
    parser.add_argument("--topic", default="/camera1/image_raw")
    parser.add_argument("--max-frames", type=int, default=240)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--warmup", type=int, default=8)
    parser.add_argument("--device", default="0")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.5)
    parser.add_argument("--output-dir", default=None)
    args = parser.parse_args()

    config_path = load_config_path(workspace, args.config)
    bag_path = Path(args.bag or config_value(config_path, "datasets", "rosbag_2026_02_05"))
    model_path = Path(args.model or config_value(config_path, "models", "yolo_runtime"))
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag not found: {bag_path}")
    if not model_path.exists():
        raise FileNotFoundError(f"YOLO model not found: {model_path}")

    output_dir = Path(args.output_dir or workspace / "log" / "eval" / f"camera_detector_benchmark_{time.strftime('%Y%m%d_%H%M%S')}")
    output_dir.mkdir(parents=True, exist_ok=True)

    import torch
    from ultralytics import YOLO

    cuda_available = bool(torch.cuda.is_available())
    device = args.device if cuda_available else "cpu"
    model = YOLO(str(model_path))

    rows: List[Dict[str, Any]] = []
    decode_ms_values: List[float] = []
    yolo_ms_values: List[float] = []
    hsv_component_ms_values: List[float] = []
    classical_ms_values: List[float] = []
    yolo_counts: List[float] = []
    hsv_component_counts: List[float] = []
    classical_counts: List[float] = []

    for frame_idx, msg in enumerate(iter_images(bag_path, args.topic, args.max_frames, args.stride)):
        decode_start = time.perf_counter()
        image_bgr = image_msg_to_bgr(msg)
        decode_ms = (time.perf_counter() - decode_start) * 1000.0

        yolo_start = time.perf_counter()
        yolo_result = model.predict(
            source=image_bgr,
            device=device,
            imgsz=args.imgsz,
            conf=args.conf,
            verbose=False,
        )[0]
        if cuda_available and device != "cpu":
            torch.cuda.synchronize()
        yolo_ms = (time.perf_counter() - yolo_start) * 1000.0
        yolo_count = 0 if yolo_result.boxes is None else len(yolo_result.boxes)

        hsv_start = time.perf_counter()
        hsv_components = detect_hsv_components(image_bgr)
        hsv_component_ms = (time.perf_counter() - hsv_start) * 1000.0

        classical_start = time.perf_counter()
        classical_detections = detect_classical_cones(image_bgr)
        classical_ms = (time.perf_counter() - classical_start) * 1000.0

        row = {
            "frame_index": frame_idx,
            "stamp": stamp_sec(msg),
            "image_width": msg.width,
            "image_height": msg.height,
            "decode_ms": decode_ms,
            "yolo_ms": yolo_ms,
            "hsv_component_ms": hsv_component_ms,
            "classical_ms": classical_ms,
            "yolo_count": yolo_count,
            "hsv_component_count": len(hsv_components),
            "classical_count": len(classical_detections),
            "is_warmup": frame_idx < args.warmup,
        }
        rows.append(row)
        if frame_idx >= args.warmup:
            decode_ms_values.append(decode_ms)
            yolo_ms_values.append(yolo_ms)
            hsv_component_ms_values.append(hsv_component_ms)
            classical_ms_values.append(classical_ms)
            yolo_counts.append(float(yolo_count))
            hsv_component_counts.append(float(len(hsv_components)))
            classical_counts.append(float(len(classical_detections)))

    with (output_dir / "camera_detector_times.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "frame_index",
                "stamp",
                "image_width",
                "image_height",
                "decode_ms",
                "yolo_ms",
                "hsv_component_ms",
                "classical_ms",
                "yolo_count",
                "hsv_component_count",
                "classical_count",
                "is_warmup",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)

    summary = {
        "bag": str(bag_path),
        "topic": args.topic,
        "model": str(model_path),
        "device": device,
        "cuda_available": cuda_available,
        "torch": getattr(torch, "__version__", None),
        "frames_total": len(rows),
        "warmup_frames": min(args.warmup, len(rows)),
        "timed_frames": len(yolo_ms_values),
        "decode_ms": stat_dict(decode_ms_values),
        "yolo_gpu_ms": stat_dict(yolo_ms_values),
        "hsv_component_cpu_ms": stat_dict(hsv_component_ms_values),
        "classical_cone_cpu_ms": stat_dict(classical_ms_values),
        "yolo_boxes_per_frame": stat_dict(yolo_counts),
        "hsv_components_per_frame": stat_dict(hsv_component_counts),
        "classical_cones_per_frame": stat_dict(classical_counts),
    }
    with (output_dir / "summary.json").open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"Output: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
