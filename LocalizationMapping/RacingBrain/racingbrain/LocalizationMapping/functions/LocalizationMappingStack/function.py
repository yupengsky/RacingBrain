from launch.actions import LogInfo

from racingbrain.LocalizationMapping.functions.Mapping.function import launch_actions as mapping_actions
from racingbrain.LocalizationMapping.functions.Perception.function import launch_actions as perception_actions
from racingbrain.LocalizationMapping.functions.Planning.function import launch_actions as planning_actions


def launch_actions(
    enable_perception,
    enable_mapping,
    enable_planning,
    eval_debug,
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
            lidar_backend=lidar_backend,
            enabled=enable_perception,
        )
    )
    actions.extend(
        mapping_actions(
            track=track,
            rviz=rviz,
            eval_debug=eval_debug,
            enabled=enable_mapping,
        )
    )
    actions.extend(planning_actions(enabled=enable_planning))
    return actions
