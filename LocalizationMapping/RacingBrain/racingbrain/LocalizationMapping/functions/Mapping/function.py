from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from racingbrain.common.launching import include_launch


def _perform(context, value):
    if hasattr(value, "perform"):
        return value.perform(context)
    return str(value)


def _enabled(context, value):
    if value is None:
        return True
    return _perform(context, value).strip().lower() in ("1", "true", "yes", "on")


def _setup(context, track, rviz, eval_debug, enabled):
    if not _enabled(context, enabled):
        return []

    return [
        include_launch(
            "slam",
            "slam.launch.py",
            {
                "track": track,
                "rviz": rviz,
                "eval_debug": eval_debug,
            },
        )
    ]


def launch_actions(
    track=None,
    rviz=None,
    eval_debug=None,
    enabled=None,
):
    track = track or LaunchConfiguration("track")
    rviz = rviz or LaunchConfiguration("rviz")
    eval_debug = eval_debug or LaunchConfiguration("eval_debug")

    return [
        OpaqueFunction(
            function=_setup,
            kwargs={
                "track": track,
                "rviz": rviz,
                "eval_debug": eval_debug,
                "enabled": enabled,
            },
        )
    ]
