from __future__ import annotations

import math
from typing import Any

from geometry_msgs.msg import Quaternion


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
