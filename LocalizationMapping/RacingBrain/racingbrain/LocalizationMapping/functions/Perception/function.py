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


def _setup(context, eval_debug, lidar_backend, enabled):
    if not _enabled(context, enabled):
        return []

    return [
        include_launch(
            "run_perception",
            "system_run.launch.py",
            {
                "eval_debug": eval_debug,
                "lidar_backend": lidar_backend,
            },
        )
    ]


def launch_actions(
    eval_debug=None,
    lidar_backend=None,
    enabled=None,
):
    eval_debug = eval_debug or LaunchConfiguration("eval_debug")
    lidar_backend = lidar_backend or LaunchConfiguration("lidar_backend")

    return [
        OpaqueFunction(
            function=_setup,
            kwargs={
                "eval_debug": eval_debug,
                "lidar_backend": lidar_backend,
                "enabled": enabled,
            },
        )
    ]
