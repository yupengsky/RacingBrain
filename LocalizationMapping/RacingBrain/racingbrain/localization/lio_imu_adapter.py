from __future__ import annotations

import math

from sensor_msgs.msg import Imu


def scale_covariance(values, scale: float):
    scaled = list(values)
    if not scaled:
        return scaled
    factor = scale * scale
    for index, value in enumerate(scaled):
        if value > 0.0 and math.isfinite(value):
            scaled[index] = value * factor
    return scaled


class LioImuAdapter:
    def __init__(self, *, acceleration_scale: float = 9.80665, gyro_scale: float = math.pi / 180.0) -> None:
        self.acceleration_scale = float(acceleration_scale)
        self.gyro_scale = float(gyro_scale)

    def adapt(self, msg: Imu) -> Imu:
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance

        out.angular_velocity.x = msg.angular_velocity.x * self.gyro_scale
        out.angular_velocity.y = msg.angular_velocity.y * self.gyro_scale
        out.angular_velocity.z = msg.angular_velocity.z * self.gyro_scale
        out.angular_velocity_covariance = scale_covariance(msg.angular_velocity_covariance, self.gyro_scale)

        out.linear_acceleration.x = msg.linear_acceleration.x * self.acceleration_scale
        out.linear_acceleration.y = msg.linear_acceleration.y * self.acceleration_scale
        out.linear_acceleration.z = msg.linear_acceleration.z * self.acceleration_scale
        out.linear_acceleration_covariance = scale_covariance(
            msg.linear_acceleration_covariance,
            self.acceleration_scale,
        )
        return out
