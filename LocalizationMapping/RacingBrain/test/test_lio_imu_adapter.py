import math

import pytest

pytest.importorskip("sensor_msgs")
from sensor_msgs.msg import Imu

from racingbrain.localization.lio_imu_adapter import LioImuAdapter


def test_lio_imu_adapter_converts_dataset_units_to_si():
    msg = Imu()
    msg.angular_velocity.z = 90.0
    msg.linear_acceleration.z = 1.0

    adapted = LioImuAdapter().adapt(msg)

    assert math.isclose(adapted.angular_velocity.z, math.pi / 2.0)
    assert math.isclose(adapted.linear_acceleration.z, 9.80665)
