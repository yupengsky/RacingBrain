import math

from racingbrain.localization.pose_sources import MultiSourcePoseJudge, Pose2D, PoseJudgeConfig


def lio_raw_from_map(pose, yaw_offset=0.2, tx=3.0, ty=-1.0):
    c = math.cos(-yaw_offset)
    s = math.sin(-yaw_offset)
    dx = pose.x - tx
    dy = pose.y - ty
    return Pose2D(
        stamp=pose.stamp,
        x=c * dx - s * dy,
        y=s * dx + c * dy,
        yaw=pose.yaw - yaw_offset,
        vx=pose.vx,
        vy=pose.vy,
        source="lio",
    )


def test_nominal_sources_are_aligned_and_fused():
    judge = MultiSourcePoseJudge(PoseJudgeConfig())
    gnss0 = Pose2D(stamp=0.0, x=0.0, y=0.0, yaw=0.1)
    judge.update_gnss(gnss0, accuracy_xy=0.1)
    judge.update_lio(lio_raw_from_map(gnss0), covariance_xy=0.02)
    first = judge.decide(0.0)
    assert first.alignment_initialized

    gnss1 = Pose2D(stamp=0.1, x=1.0, y=0.2, yaw=0.12)
    judge.update_gnss(gnss1, accuracy_xy=0.1)
    judge.update_lio(lio_raw_from_map(gnss1), covariance_xy=0.02)
    decision = judge.decide(0.1)

    assert decision.source == "fused"
    assert decision.state == "nominal"
    assert decision.cross_position_error_m is not None
    assert decision.cross_position_error_m < 1e-6


def test_degraded_gnss_uses_lio_after_alignment():
    judge = MultiSourcePoseJudge(PoseJudgeConfig(fusion_enabled=False))
    gnss0 = Pose2D(stamp=0.0, x=0.0, y=0.0, yaw=0.0)
    judge.update_gnss(gnss0, accuracy_xy=0.1)
    judge.update_lio(lio_raw_from_map(gnss0), covariance_xy=0.02)
    judge.decide(0.0)

    true_pose = Pose2D(stamp=0.1, x=1.0, y=0.0, yaw=0.0)
    bad_gnss = Pose2D(stamp=0.1, x=4.0, y=-1.5, yaw=0.0)
    judge.update_gnss(bad_gnss, accuracy_xy=3.5)
    judge.update_lio(lio_raw_from_map(true_pose), covariance_xy=0.02)
    decision = judge.decide(0.1)

    assert decision.source == "lio"
    assert decision.state in {"degraded", "conflict"}
    assert decision.pose is not None
    assert decision.pose.distance_to(true_pose) < 1e-6


def test_lio_stale_falls_back_to_gnss():
    judge = MultiSourcePoseJudge(PoseJudgeConfig(stale_timeout_sec=0.2, fusion_enabled=False))
    gnss0 = Pose2D(stamp=0.0, x=0.0, y=0.0, yaw=0.0)
    judge.update_gnss(gnss0, accuracy_xy=0.1)
    judge.update_lio(lio_raw_from_map(gnss0), covariance_xy=0.02)
    judge.decide(0.0)

    gnss1 = Pose2D(stamp=1.0, x=1.0, y=0.0, yaw=0.0)
    judge.update_gnss(gnss1, accuracy_xy=0.1)
    decision = judge.decide(1.0)

    assert decision.source == "gnss_ins"
    assert decision.state == "degraded"
