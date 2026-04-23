#!/usr/bin/env python3
"""Generate a perturbed fusion calibration file for replay experiments."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import yaml


def rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, help="Source calibration YAML")
    parser.add_argument("--output", required=True, help="Output calibration YAML")
    parser.add_argument("--translation-x", type=float, default=0.0)
    parser.add_argument("--translation-y", type=float, default=0.0)
    parser.add_argument("--translation-z", type=float, default=0.0)
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, default=0.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source_path = Path(args.source)
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    data = yaml.safe_load(source_path.read_text(encoding="utf-8"))
    params = data["/fusion_box_node"]["ros__parameters"]
    matrix = np.array(params["lidar_to_camera_matrix"], dtype=float).reshape(4, 4)

    delta_r = rotation_matrix(args.roll_deg, args.pitch_deg, args.yaw_deg)
    matrix[:3, :3] = delta_r @ matrix[:3, :3]
    matrix[0, 3] += args.translation_x
    matrix[1, 3] += args.translation_y
    matrix[2, 3] += args.translation_z

    params["lidar_to_camera_matrix"] = [round(float(v), 10) for v in matrix.reshape(-1).tolist()]
    output_path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
