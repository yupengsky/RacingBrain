from launch.actions import LogInfo

from racingbrain.functions.health.function import launch_actions as health_actions
from racingbrain.functions.mapping.function import launch_actions as mapping_actions
from racingbrain.functions.perception.function import launch_actions as perception_actions
from racingbrain.functions.planning.function import launch_actions as planning_actions


def launch_actions(
    enable_perception,
    enable_mapping,
    enable_planning,
    enable_health,
    eval_debug,
    health_period,
    health_stale_timeout,
    camera_topic,
    lidar_topic,
    gnss_topic,
    fusion_calibration_file,
    lidar_backend,
    lidar_verifier,
    fusion_mode,
    mapping_gate,
    track,
    rviz,
    health_expected_perception,
):
    actions = [
        LogInfo(msg="RacingBrain real-time localization and mapping stack is starting."),
    ]
    actions.extend(
        perception_actions(
            eval_debug=eval_debug,
            health_metrics=enable_health,
            camera_topic=camera_topic,
            lidar_topic=lidar_topic,
            fusion_calibration_file=fusion_calibration_file,
            lidar_backend=lidar_backend,
            lidar_verifier=lidar_verifier,
            fusion_mode=fusion_mode,
            enabled=enable_perception,
        )
    )
    actions.extend(
        mapping_actions(
            track=track,
            rviz=rviz,
            eval_debug=eval_debug,
            health_metrics=enable_health,
            gnss_topic=gnss_topic,
            mapping_gate=mapping_gate,
            enabled=enable_mapping,
        )
    )
    actions.extend(
        health_actions(
            expected_perception=health_expected_perception,
            expected_mapping=enable_mapping,
            enabled=enable_health,
            lidar_backend=lidar_backend,
            publish_period_sec=health_period,
            stale_timeout_sec=health_stale_timeout,
        )
    )
    actions.extend(planning_actions(enabled=enable_planning))
    return actions
