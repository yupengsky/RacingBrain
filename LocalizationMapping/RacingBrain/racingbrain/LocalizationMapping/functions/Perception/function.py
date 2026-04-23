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


def _setup(context, eval_debug, health_metrics, camera_topic, lidar_topic, fusion_calibration_file, lidar_backend, enabled):
    if not _enabled(context, enabled):
        return []

    launch_args = {
        "eval_debug": eval_debug,
        "health_metrics": health_metrics,
        "camera_topic": camera_topic,
        "lidar_topic": lidar_topic,
        "lidar_backend": lidar_backend,
    }
    calibration_value = _perform(context, fusion_calibration_file).strip()
    if calibration_value:
        launch_args["fusion_calibration_file"] = calibration_value

    return [
        include_launch(
            "run_perception",
            "system_run.launch.py",
            launch_args,
        )
    ]


def launch_actions(
    eval_debug=None,
    health_metrics=None,
    camera_topic=None,
    lidar_topic=None,
    fusion_calibration_file=None,
    lidar_backend=None,
    enabled=None,
):
    eval_debug = eval_debug or LaunchConfiguration("eval_debug")
    health_metrics = health_metrics or LaunchConfiguration("enable_health")
    camera_topic = camera_topic or LaunchConfiguration("camera_topic")
    lidar_topic = lidar_topic or LaunchConfiguration("lidar_topic")
    fusion_calibration_file = fusion_calibration_file or LaunchConfiguration("fusion_calibration_file")
    lidar_backend = lidar_backend or LaunchConfiguration("lidar_backend")

    return [
        OpaqueFunction(
            function=_setup,
            kwargs={
                "eval_debug": eval_debug,
                "health_metrics": health_metrics,
                "camera_topic": camera_topic,
                "lidar_topic": lidar_topic,
                "fusion_calibration_file": fusion_calibration_file,
                "lidar_backend": lidar_backend,
                "enabled": enabled,
            },
        )
    ]
