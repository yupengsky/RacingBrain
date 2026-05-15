from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch_ros.actions import Node


def launch_actions(enabled=None):
    condition = IfCondition(enabled) if enabled is not None else None
    return [
        LogInfo(
            msg=(
                "RacingBrain planning interface enabled: extracting sparse track graph from /global_map."
            ),
            condition=condition,
        ),
        Node(
            package="racingbrain",
            executable="track_graph_builder",
            name="track_graph_builder",
            output="screen",
            condition=condition,
        ),
    ]
