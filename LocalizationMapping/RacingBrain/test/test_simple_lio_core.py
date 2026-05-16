import math

import numpy as np
import pytest

pytest.importorskip("sensor_msgs")
pytest.importorskip("sensor_msgs_py")
pytest.importorskip("scipy")
from racingbrain.localization.simple_lio_core import (
    OnlineImuYawBuffer,
    Pose2,
    SimpleLioConfig,
    run_icp,
    transform_points,
)


def test_online_imu_yaw_buffer_integrates_scaled_rate():
    imu = OnlineImuYawBuffer(gyro_scale=2.0)
    imu.add(1.0, 1.0)
    imu.add(1.1, 1.0)

    assert math.isclose(imu.yaw_delta(1.0, 1.1), 0.2)


def test_simple_lio_icp_recovers_se2_pose():
    scan = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.8, 0.2, 0.0],
            [1.7, -0.1, 0.0],
            [0.3, 1.3, 0.0],
            [1.4, 1.1, 0.0],
            [2.2, 0.7, 0.0],
            [-0.2, 2.0, 0.0],
        ],
        dtype=np.float64,
    )
    truth = Pose2(stamp=2.0, x=1.2, y=-0.4, yaw=0.25)
    local_map = transform_points(scan, truth)
    cfg = SimpleLioConfig(
        icp_min_inliers=3,
        icp_iterations=5,
        icp_max_correspondence_m=5.0,
        icp_max_translation_step_m=5.0,
        icp_max_yaw_step_rad=1.0,
        icp_max_pose_correction_m=5.0,
        icp_max_pose_correction_yaw_rad=1.0,
    )

    initial = Pose2(stamp=2.0, x=1.0, y=-0.3, yaw=0.2)

    result = run_icp(scan, local_map, initial, cfg)

    assert result.used
    assert math.isclose(result.pose.x, truth.x, abs_tol=1e-6)
    assert math.isclose(result.pose.y, truth.y, abs_tol=1e-6)
    assert math.isclose(result.pose.yaw, truth.yaw, abs_tol=1e-6)
