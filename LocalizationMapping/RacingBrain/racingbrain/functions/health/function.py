from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_actions(
    expected_perception=None,
    expected_mapping=None,
    enabled=None,
    lidar_backend=None,
    publish_period_sec=None,
    stale_timeout_sec=None,
):
    expected_perception = expected_perception or LaunchConfiguration("enable_perception")
    expected_mapping = expected_mapping or LaunchConfiguration("enable_mapping")
    enabled = enabled or LaunchConfiguration("enable_health")
    lidar_backend = lidar_backend or LaunchConfiguration("lidar_backend")
    publish_period_sec = publish_period_sec or LaunchConfiguration("health_period")
    stale_timeout_sec = stale_timeout_sec or LaunchConfiguration("health_stale_timeout")

    return [
        Node(
            package="racingbrain",
            executable="system_health_monitor",
            name="system_health_monitor",
            output="screen",
            condition=IfCondition(enabled),
            parameters=[
                {
                    "expected_perception": ParameterValue(expected_perception, value_type=bool),
                    "expected_mapping": ParameterValue(expected_mapping, value_type=bool),
                    "selected_lidar_backend": lidar_backend,
                    "publish_period_sec": ParameterValue(publish_period_sec, value_type=float),
                    "stale_timeout_sec": ParameterValue(stale_timeout_sec, value_type=float),
                }
            ],
        )
    ]
