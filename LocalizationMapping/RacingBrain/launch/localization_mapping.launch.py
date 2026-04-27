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
    camera_topic = LaunchConfiguration("camera_topic")
    lidar_topic = LaunchConfiguration("lidar_topic")
    gnss_topic = LaunchConfiguration("gnss_topic")
    fusion_calibration_file = LaunchConfiguration("fusion_calibration_file")
    lidar_backend = LaunchConfiguration("lidar_backend")
    lidar_verifier = LaunchConfiguration("lidar_verifier")
    mapping_gate = LaunchConfiguration("mapping_gate")
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
            DeclareLaunchArgument("camera_topic", default_value="/camera1/image_raw", description="Camera topic for YOLO input."),
            DeclareLaunchArgument("lidar_topic", default_value="/lidar_points", description="LiDAR point-cloud topic for cone detection."),
            DeclareLaunchArgument("gnss_topic", default_value="/gongji_gnss_ins_64", description="GNSS/INS topic for mapping."),
            DeclareLaunchArgument("fusion_calibration_file", default_value="", description="Optional override for the fusion calibration YAML file."),
            DeclareLaunchArgument("lidar_backend", default_value="pointpillars", description="LiDAR backend: pointpillars, cluster, or auto."),
            DeclareLaunchArgument("lidar_verifier", default_value="true", description="Enable local verification for PointPillars output."),
            DeclareLaunchArgument("mapping_gate", default_value="true", description="Enable risk-aware mapping gate."),
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
                camera_topic=camera_topic,
                lidar_topic=lidar_topic,
                gnss_topic=gnss_topic,
                fusion_calibration_file=fusion_calibration_file,
                lidar_backend=lidar_backend,
                lidar_verifier=lidar_verifier,
                mapping_gate=mapping_gate,
                track=track,
                rviz=rviz,
            ),
        ]
    )
