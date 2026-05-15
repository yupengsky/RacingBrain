from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gnss_topic = LaunchConfiguration("gnss_topic")
    lio_odom_topic = LaunchConfiguration("lio_odom_topic")
    output_odom_topic = LaunchConfiguration("output_odom_topic")
    output_gnss_topic = LaunchConfiguration("output_gnss_topic")
    diagnostics_topic = LaunchConfiguration("diagnostics_topic")
    fusion_enabled = LaunchConfiguration("fusion_enabled")
    publish_compat_gnss = LaunchConfiguration("publish_compat_gnss")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gnss_topic", default_value="/gongji_gnss_ins_64"),
            DeclareLaunchArgument("lio_odom_topic", default_value="/lio_sam/mapping/odometry"),
            DeclareLaunchArgument("output_odom_topic", default_value="/racingbrain/localization/pose"),
            DeclareLaunchArgument("output_gnss_topic", default_value="/racingbrain/localization/gnss_ins_pose"),
            DeclareLaunchArgument("diagnostics_topic", default_value="/racingbrain/localization/pose_judge"),
            DeclareLaunchArgument("fusion_enabled", default_value="true"),
            DeclareLaunchArgument("publish_compat_gnss", default_value="true"),
            Node(
                package="racingbrain",
                executable="multisource_pose_judge",
                name="multisource_pose_judge",
                output="screen",
                parameters=[
                    {
                        "gnss_topic": gnss_topic,
                        "lio_odom_topic": lio_odom_topic,
                        "output_odom_topic": output_odom_topic,
                        "output_gnss_topic": output_gnss_topic,
                        "diagnostics_topic": diagnostics_topic,
                        "fusion_enabled": ParameterValue(fusion_enabled, value_type=bool),
                        "publish_compat_gnss": ParameterValue(publish_compat_gnss, value_type=bool),
                    }
                ],
            ),
        ]
    )
