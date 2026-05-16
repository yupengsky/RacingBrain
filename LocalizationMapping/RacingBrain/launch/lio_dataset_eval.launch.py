from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    run_simple_lio = LaunchConfiguration("run_simple_lio")
    run_pointcloud_adapter = LaunchConfiguration("run_pointcloud_adapter")
    run_imu_adapter = LaunchConfiguration("run_imu_adapter")
    run_error_eval = LaunchConfiguration("run_error_eval")
    input_cloud_topic = LaunchConfiguration("input_cloud_topic")
    adapted_cloud_topic = LaunchConfiguration("adapted_cloud_topic")
    lidar_frame = LaunchConfiguration("lidar_frame")
    gnss_topic = LaunchConfiguration("gnss_topic")
    input_imu_topic = LaunchConfiguration("input_imu_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    lio_odom_topic = LaunchConfiguration("lio_odom_topic")
    output_dir = LaunchConfiguration("output_dir")
    scan_period_sec = LaunchConfiguration("scan_period_sec")
    n_scan = LaunchConfiguration("n_scan")
    imu_gyro_scale = LaunchConfiguration("imu_gyro_scale")
    position_warn_m = LaunchConfiguration("position_warn_m")
    yaw_warn_rad = LaunchConfiguration("yaw_warn_rad")

    return LaunchDescription(
        [
            DeclareLaunchArgument("run_simple_lio", default_value="true"),
            DeclareLaunchArgument("run_pointcloud_adapter", default_value="false"),
            DeclareLaunchArgument("run_imu_adapter", default_value="false"),
            DeclareLaunchArgument("run_error_eval", default_value="true"),
            DeclareLaunchArgument("input_cloud_topic", default_value="/lidar_points"),
            DeclareLaunchArgument("adapted_cloud_topic", default_value="/points"),
            DeclareLaunchArgument("lidar_frame", default_value="base_link"),
            DeclareLaunchArgument("gnss_topic", default_value="/gongji_gnss_ins_64"),
            DeclareLaunchArgument("input_imu_topic", default_value="/imu"),
            DeclareLaunchArgument("imu_topic", default_value="/imu_lio"),
            DeclareLaunchArgument("lio_odom_topic", default_value="/racingbrain/simple_lio/odometry"),
            DeclareLaunchArgument("output_dir", default_value="log/benchmark/lio_gnss/latest"),
            DeclareLaunchArgument("scan_period_sec", default_value="0.1"),
            DeclareLaunchArgument("n_scan", default_value="64"),
            DeclareLaunchArgument("imu_gyro_scale", default_value="0.04348764102608839"),
            DeclareLaunchArgument("position_warn_m", default_value="1.0"),
            DeclareLaunchArgument("yaw_warn_rad", default_value="0.25"),
            Node(
                package="racingbrain",
                executable="lio_pointcloud_adapter",
                name="lio_pointcloud_adapter",
                output="screen",
                condition=IfCondition(run_pointcloud_adapter),
                parameters=[
                    {
                        "input_cloud_topic": input_cloud_topic,
                        "output_cloud_topic": adapted_cloud_topic,
                        "output_frame_id": lidar_frame,
                        "scan_period_sec": ParameterValue(scan_period_sec, value_type=float),
                        "n_scan": ParameterValue(n_scan, value_type=int),
                    }
                ],
            ),
            Node(
                package="racingbrain",
                executable="lio_imu_adapter",
                name="lio_imu_adapter",
                output="screen",
                condition=IfCondition(run_imu_adapter),
                parameters=[
                    {
                        "input_imu_topic": input_imu_topic,
                        "output_imu_topic": imu_topic,
                    }
                ],
            ),
            Node(
                package="racingbrain",
                executable="simple_lio",
                name="simple_lio",
                parameters=[
                    {
                        "cloud_topic": input_cloud_topic,
                        "imu_topic": input_imu_topic,
                        "gnss_topic": gnss_topic,
                        "odom_topic": lio_odom_topic,
                        "base_frame": lidar_frame,
                        "imu_gyro_scale": ParameterValue(imu_gyro_scale, value_type=float),
                    }
                ],
                output="screen",
                condition=IfCondition(run_simple_lio),
            ),
            Node(
                package="racingbrain",
                executable="lio_gnss_error_eval",
                name="lio_gnss_error_eval",
                output="screen",
                condition=IfCondition(run_error_eval),
                parameters=[
                    {
                        "gnss_topic": gnss_topic,
                        "lio_odom_topic": lio_odom_topic,
                        "output_dir": output_dir,
                        "position_warn_m": ParameterValue(position_warn_m, value_type=float),
                        "yaw_warn_rad": ParameterValue(yaw_warn_rad, value_type=float),
                    }
                ],
            ),
        ]
    )
