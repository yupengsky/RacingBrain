from __future__ import annotations

import array
import math
from dataclasses import dataclass
from typing import Dict, Iterable, Optional

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


VELODYNE_POINT_STEP = 32
VELODYNE_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name="ring", offset=20, datatype=PointField.UINT16, count=1),
    PointField(name="time", offset=24, datatype=PointField.FLOAT32, count=1),
]
VELODYNE_DTYPE = point_cloud2.dtype_from_fields(VELODYNE_FIELDS, point_step=VELODYNE_POINT_STEP)


def field_map(msg: PointCloud2) -> Dict[str, PointField]:
    return {field.name: field for field in msg.fields if field.name}


def choose_first_field(names: Iterable[str], available: Dict[str, PointField]) -> Optional[str]:
    for name in names:
        if name in available:
            return name
    return None


def normalize_time(values: np.ndarray, scan_period_sec: float) -> np.ndarray:
    if values.size == 0:
        return values.astype(np.float32)

    raw = values.astype(np.float64, copy=False)
    finite = np.isfinite(raw)
    if not np.any(finite):
        return np.zeros(values.shape, dtype=np.float32)

    clean = np.where(finite, raw, np.nanmin(raw[finite]))
    span = float(np.nanmax(clean) - np.nanmin(clean))
    shifted = clean - float(np.nanmin(clean))
    if span <= max(scan_period_sec * 2.0, 1e-6):
        rel = shifted
    elif span * 1e-9 <= scan_period_sec * 2.0:
        rel = shifted * 1e-9
    elif span * 1e-6 <= scan_period_sec * 2.0:
        rel = shifted * 1e-6
    elif span * 1e-3 <= scan_period_sec * 2.0:
        rel = shifted * 1e-3
    else:
        rel = shifted / max(span, 1e-9) * scan_period_sec
    return np.clip(rel, 0.0, scan_period_sec).astype(np.float32)


def infer_time_from_azimuth(x: np.ndarray, y: np.ndarray, scan_period_sec: float) -> np.ndarray:
    if x.size == 0:
        return np.zeros(x.shape, dtype=np.float32)
    azimuth = np.mod(np.arctan2(y.astype(np.float64), x.astype(np.float64)), 2.0 * math.pi)
    start = float(azimuth[0]) if azimuth.size else 0.0
    rel = np.mod(azimuth - start, 2.0 * math.pi) / (2.0 * math.pi) * scan_period_sec
    return rel.astype(np.float32)


def infer_ring_from_vertical_angle(
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    *,
    n_scan: int,
    vertical_fov_lower_deg: float,
    vertical_fov_upper_deg: float,
) -> np.ndarray:
    if x.size == 0:
        return np.zeros(x.shape, dtype=np.uint16)
    horizontal = np.hypot(x.astype(np.float64), y.astype(np.float64))
    angle = np.degrees(np.arctan2(z.astype(np.float64), np.maximum(horizontal, 1e-9)))
    span = max(vertical_fov_upper_deg - vertical_fov_lower_deg, 1e-6)
    ring = np.rint((angle - vertical_fov_lower_deg) / span * float(max(n_scan - 1, 1)))
    return np.clip(ring, 0, max(n_scan - 1, 0)).astype(np.uint16)


@dataclass
class AdaptStats:
    input_points: int
    output_points: int
    ring_source: str
    time_source: str
    frame_id: str

    def as_dict(self) -> Dict[str, object]:
        return {
            "input_points": self.input_points,
            "output_points": self.output_points,
            "ring_source": self.ring_source,
            "time_source": self.time_source,
            "frame_id": self.frame_id,
        }


class VelodynePointCloudAdapter:
    def __init__(
        self,
        *,
        output_frame_id: str = "lidar_link",
        scan_period_sec: float = 0.1,
        n_scan: int = 64,
        vertical_fov_lower_deg: float = -25.0,
        vertical_fov_upper_deg: float = 15.0,
        missing_ring_policy: str = "infer",
        missing_time_policy: str = "azimuth",
        drop_invalid_points: bool = True,
    ) -> None:
        self.output_frame_id = output_frame_id
        self.scan_period_sec = float(scan_period_sec)
        self.n_scan = int(n_scan)
        self.vertical_fov_lower_deg = float(vertical_fov_lower_deg)
        self.vertical_fov_upper_deg = float(vertical_fov_upper_deg)
        self.missing_ring_policy = missing_ring_policy
        self.missing_time_policy = missing_time_policy
        self.drop_invalid_points = bool(drop_invalid_points)

    def adapt(self, msg: PointCloud2) -> tuple[Optional[PointCloud2], AdaptStats]:
        available = field_map(msg)
        for required in ("x", "y", "z"):
            if required not in available:
                raise ValueError(f"PointCloud2 is missing required field: {required}")

        dtype = point_cloud2.dtype_from_fields(msg.fields, point_step=msg.point_step)
        points = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
        x = points["x"].astype(np.float32, copy=False)
        y = points["y"].astype(np.float32, copy=False)
        z = points["z"].astype(np.float32, copy=False)

        mask = np.ones(points.shape[0], dtype=bool)
        if self.drop_invalid_points:
            mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)

        intensity_name = choose_first_field(("intensity", "reflectivity", "i"), available)
        if intensity_name is None:
            intensity = np.zeros(points.shape[0], dtype=np.float32)
        else:
            intensity = points[intensity_name].astype(np.float32, copy=False)

        ring_name = choose_first_field(("ring", "channel", "laser_id", "scan_id"), available)
        if ring_name is not None:
            ring = points[ring_name].astype(np.uint16, copy=False)
            ring_source = ring_name
        elif self.missing_ring_policy == "infer":
            ring = infer_ring_from_vertical_angle(
                x,
                y,
                z,
                n_scan=self.n_scan,
                vertical_fov_lower_deg=self.vertical_fov_lower_deg,
                vertical_fov_upper_deg=self.vertical_fov_upper_deg,
            )
            ring_source = "vertical_angle"
        elif self.missing_ring_policy == "zero":
            ring = np.zeros(points.shape[0], dtype=np.uint16)
            ring_source = "zero"
        else:
            raise ValueError("PointCloud2 has no ring-like field and missing_ring_policy is not infer/zero")

        time_name = choose_first_field(("time", "t", "timestamp", "offset_time"), available)
        if time_name is not None:
            time_values = normalize_time(points[time_name], self.scan_period_sec)
            time_source = time_name
        elif self.missing_time_policy == "azimuth":
            time_values = infer_time_from_azimuth(x, y, self.scan_period_sec)
            time_source = "azimuth"
        elif self.missing_time_policy == "zero":
            time_values = np.zeros(points.shape[0], dtype=np.float32)
            time_source = "zero"
        else:
            raise ValueError("PointCloud2 has no time-like field and missing_time_policy is not azimuth/zero")

        if not np.any(mask):
            stats = AdaptStats(points.shape[0], 0, ring_source, time_source, self.output_frame_id or msg.header.frame_id)
            return None, stats

        out = np.zeros(int(np.count_nonzero(mask)), dtype=VELODYNE_DTYPE)
        out["x"] = x[mask]
        out["y"] = y[mask]
        out["z"] = z[mask]
        out["intensity"] = intensity[mask]
        out["ring"] = ring[mask]
        out["time"] = time_values[mask]

        cloud = PointCloud2()
        cloud.header = msg.header
        if self.output_frame_id:
            cloud.header.frame_id = self.output_frame_id
        cloud.height = 1
        cloud.width = out.shape[0]
        cloud.fields = VELODYNE_FIELDS
        cloud.is_bigendian = False
        cloud.point_step = VELODYNE_POINT_STEP
        cloud.row_step = VELODYNE_POINT_STEP * out.shape[0]
        cloud.is_dense = True
        cloud.data = array.array("B", out.tobytes())

        stats = AdaptStats(
            input_points=points.shape[0],
            output_points=out.shape[0],
            ring_source=ring_source,
            time_source=time_source,
            frame_id=cloud.header.frame_id,
        )
        return cloud, stats
