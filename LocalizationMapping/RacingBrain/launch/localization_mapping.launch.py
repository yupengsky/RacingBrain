from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from racingbrain.LocalizationMapping.functions.LocalizationMappingStack.function import launch_actions


def generate_launch_description():
    enable_perception = LaunchConfiguration("enable_perception")
    enable_mapping = LaunchConfiguration("enable_mapping")
    enable_planning = LaunchConfiguration("enable_planning")
    eval_debug = LaunchConfiguration("eval_debug")
    lidar_backend = LaunchConfiguration("lidar_backend")
    track = LaunchConfiguration("track")
    rviz = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_perception", default_value="true", description="Start real-time perception."),
            DeclareLaunchArgument("enable_mapping", default_value="true", description="Start GNSS/INS-aided localization and mapping."),
            DeclareLaunchArgument("enable_planning", default_value="false", description="Enable reserved planner hook."),
            DeclareLaunchArgument("eval_debug", default_value="false", description="Enable debug evaluation topics."),
            DeclareLaunchArgument("lidar_backend", default_value="pointpillars", description="LiDAR backend: pointpillars or cluster."),
            DeclareLaunchArgument("track", default_value="acceleration", description="Track config: acceleration, autocross, or skidpad."),
            DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz with mapping."),
            *launch_actions(
                enable_perception=enable_perception,
                enable_mapping=enable_mapping,
                enable_planning=enable_planning,
                eval_debug=eval_debug,
                lidar_backend=lidar_backend,
                track=track,
                rviz=rviz,
            ),
        ]
    )
