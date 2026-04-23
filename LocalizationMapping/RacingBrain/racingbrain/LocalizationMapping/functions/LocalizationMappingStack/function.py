from launch.actions import LogInfo

from racingbrain.LocalizationMapping.functions.Health.function import launch_actions as health_actions
from racingbrain.LocalizationMapping.functions.Mapping.function import launch_actions as mapping_actions
from racingbrain.LocalizationMapping.functions.Perception.function import launch_actions as perception_actions
from racingbrain.LocalizationMapping.functions.Planning.function import launch_actions as planning_actions


def launch_actions(
    enable_perception,
    enable_mapping,
    enable_planning,
    enable_health,
    eval_debug,
    health_period,
    health_stale_timeout,
    lidar_backend,
    track,
    rviz,
):
    actions = [
        LogInfo(msg="RacingBrain real-time localization and mapping stack is starting."),
    ]
    actions.extend(
        perception_actions(
            eval_debug=eval_debug,
            health_metrics=enable_health,
            lidar_backend=lidar_backend,
            enabled=enable_perception,
        )
    )
    actions.extend(
        mapping_actions(
            track=track,
            rviz=rviz,
            eval_debug=eval_debug,
            health_metrics=enable_health,
            enabled=enable_mapping,
        )
    )
    actions.extend(
        health_actions(
            expected_perception=enable_perception,
            expected_mapping=enable_mapping,
            enabled=enable_health,
            lidar_backend=lidar_backend,
            publish_period_sec=health_period,
            stale_timeout_sec=health_stale_timeout,
        )
    )
    actions.extend(planning_actions(enabled=enable_planning))
    return actions
