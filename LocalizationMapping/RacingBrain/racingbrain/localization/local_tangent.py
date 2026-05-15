from __future__ import annotations

import math
from typing import Optional


EARTH_RADIUS_M = 6378137.0


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
