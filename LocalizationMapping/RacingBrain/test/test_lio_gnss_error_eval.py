import math

from racingbrain.localization.lio_gnss_error_eval import GnssSample, LioGnssErrorEvaluator
from racingbrain.localization.pose_sources import Pose2D


def test_lio_evaluator_aligns_first_pair_and_reports_drift():
    evaluator = LioGnssErrorEvaluator(sync_tolerance_sec=0.1)
    evaluator.update_gnss(GnssSample(Pose2D(stamp=0.0, x=10.0, y=5.0, yaw=0.2), accuracy_xy=0.1))
    first = evaluator.update_lio(Pose2D(stamp=0.0, x=0.0, y=0.0, yaw=0.0))

    assert first is not None
    assert first.position_error_m < 1e-9

    evaluator.update_gnss(GnssSample(Pose2D(stamp=0.1, x=11.0, y=5.0, yaw=0.2), accuracy_xy=0.1))
    sample = evaluator.update_lio(Pose2D(stamp=0.1, x=1.5, y=0.0, yaw=0.0))

    assert sample is not None
    assert 0.55 < sample.position_error_m < 0.57
    summary = evaluator.summary(position_warn_m=0.4, yaw_warn_rad=0.2)
    assert summary["sample_count"] == 2
    assert summary["position_warn_count"] == 1


def test_lio_evaluator_rejects_unsynchronized_samples():
    evaluator = LioGnssErrorEvaluator(sync_tolerance_sec=0.01)
    evaluator.update_gnss(GnssSample(Pose2D(stamp=1.0, x=0.0, y=0.0, yaw=0.0), accuracy_xy=None))

    sample = evaluator.update_lio(Pose2D(stamp=1.2, x=0.0, y=0.0, yaw=math.pi))

    assert sample is None
    assert not evaluator.samples
