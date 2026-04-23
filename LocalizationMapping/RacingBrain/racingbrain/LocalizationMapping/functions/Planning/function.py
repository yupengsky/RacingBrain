from launch.actions import LogInfo
from launch.conditions import IfCondition


def launch_actions(enabled=None):
    condition = IfCondition(enabled) if enabled is not None else None
    return [
        LogInfo(
            msg=(
                "RacingBrain planner hook is reserved. "
                "Add planner launch actions here when the planning module is ready."
            ),
            condition=condition,
        )
    ]
