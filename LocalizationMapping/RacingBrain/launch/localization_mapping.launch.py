from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from racingbrain.LocalizationMapping.functions.LocalizationMappingStack.function import launch_actions


def generate_launch_description():
    enable_perception = LaunchConfiguration("enable_perception")
    enable_mapping = LaunchConfiguration("enable_mapping")
    enable_planning = LaunchConfiguration("enable_planning")
    enable_health = LaunchConfiguration("enable_health")
    eval_debug = LaunchConfiguration("eval_debug")
    health_period = LaunchConfiguration("health_period")
    health_stale_timeout = LaunchConfiguration("health_stale_timeout")
    lidar_backend = LaunchConfiguration("lidar_backend")
    track = LaunchConfiguration("track")
    rviz = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_perception", default_value="true", description="Start real-time perception."),
            DeclareLaunchArgument("enable_mapping", default_value="true", description="Start GNSS/INS-aided localization and mapping."),
            DeclareLaunchArgument("enable_planning", default_value="false", description="Enable reserved planner hook."),
            DeclareLaunchArgument("enable_health", default_value="true", description="Start the online system health monitor."),
            DeclareLaunchArgument("eval_debug", default_value="false", description="Enable debug evaluation topics."),
            DeclareLaunchArgument("health_period", default_value="1.0", description="System health publish period in seconds."),
            DeclareLaunchArgument("health_stale_timeout", default_value="3.0", description="Mark a component stale when no fresh metrics arrive within this timeout."),
            DeclareLaunchArgument("lidar_backend", default_value="pointpillars", description="LiDAR backend: pointpillars or cluster."),
            DeclareLaunchArgument("track", default_value="acceleration", description="Track config: acceleration, autocross, or skidpad."),
            DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz with mapping."),
            *launch_actions(
                enable_perception=enable_perception,
                enable_mapping=enable_mapping,
                enable_planning=enable_planning,
                enable_health=enable_health,
                eval_debug=eval_debug,
                health_period=health_period,
                health_stale_timeout=health_stale_timeout,
                lidar_backend=lidar_backend,
                track=track,
                rviz=rviz,
            ),
        ]
    )
