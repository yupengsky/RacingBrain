import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("racing_control_bringup")

    config_autocross = os.path.join(package_share, "config", "config_autocross.yaml")
    config_skidpad = os.path.join(package_share, "config", "config_skidpad.yaml")
    config_acceleration = os.path.join(package_share, "config", "config_acceleration.yaml")

    mode = LaunchConfiguration("mode")
    config_file = PythonExpression(
        [
            "'",
            config_skidpad,
            "' if '",
            mode,
            "' == 'skidpad' else '",
            config_acceleration,
            "' if '",
            mode,
            "' == 'acceleration' else '",
            config_autocross,
            "'",
        ]
    )

    planner_common_params = {
        "mode": LaunchConfiguration("mode"),
        "rviz_visualization": LaunchConfiguration("rviz_visualization"),
        "map_topic": LaunchConfiguration("planner_map_topic"),
        "odom_topic": LaunchConfiguration("planner_odom_topic"),
        "path_topic": LaunchConfiguration("path_topic"),
        "state_indicator_topic": LaunchConfiguration("state_indicator_topic"),
        "off_track_topic": LaunchConfiguration("off_track_topic"),
        "brake_topic": LaunchConfiguration("brake_topic"),
    }
    controller_common_params = {
        "odom_topic": LaunchConfiguration("planner_odom_topic"),
        "path_topic": LaunchConfiguration("path_topic"),
        "state_indicator_topic": LaunchConfiguration("state_indicator_topic"),
        "off_track_topic": LaunchConfiguration("off_track_topic"),
        "brake_topic": LaunchConfiguration("brake_topic"),
        "control_topic": LaunchConfiguration("control_topic"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="autocross"),
            DeclareLaunchArgument("dummy_control", default_value="false"),
            DeclareLaunchArgument("rviz_visualization", default_value="false"),
            DeclareLaunchArgument("use_racingbrain_adapters", default_value="true"),
            DeclareLaunchArgument("use_fsds_track_adapter", default_value="false"),
            DeclareLaunchArgument("config_file", default_value=config_file),
            DeclareLaunchArgument("racingbrain_map_topic", default_value="/global_map"),
            DeclareLaunchArgument("racingbrain_odom_topic", default_value="/vehicle_odom"),
            DeclareLaunchArgument("fsds_track_topic", default_value="/testing_only/track"),
            DeclareLaunchArgument("fsds_track_transient_local", default_value="false"),
            DeclareLaunchArgument("planner_map_topic", default_value="/drd25/map"),
            DeclareLaunchArgument("planner_odom_topic", default_value="/testing_only/odom"),
            DeclareLaunchArgument("path_topic", default_value="/drd25/path"),
            DeclareLaunchArgument("state_indicator_topic", default_value="/drd25/state_indicator"),
            DeclareLaunchArgument("off_track_topic", default_value="/drd25/off_track"),
            DeclareLaunchArgument("brake_topic", default_value="/drd25/brake_command"),
            DeclareLaunchArgument("control_topic", default_value="/control_command"),
            Node(
                package="racing_control_adapters",
                executable="racingbrain_map_odom_adapter",
                name="racingbrain_map_odom_adapter",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_racingbrain_adapters")),
                parameters=[
                    {
                        "marker_map_topic": LaunchConfiguration("racingbrain_map_topic"),
                        "planner_map_topic": LaunchConfiguration("planner_map_topic"),
                        "odom_input_topic": LaunchConfiguration("racingbrain_odom_topic"),
                        "planner_odom_topic": LaunchConfiguration("planner_odom_topic"),
                    }
                ],
            ),
            Node(
                package="racing_control_adapters",
                executable="fsds_track_to_map_adapter",
                name="fsds_track_to_map_adapter",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_fsds_track_adapter")),
                parameters=[
                    {
                        "fsds_track_topic": LaunchConfiguration("fsds_track_topic"),
                        "planner_map_topic": LaunchConfiguration("planner_map_topic"),
                        "fsds_track_transient_local": LaunchConfiguration("fsds_track_transient_local"),
                    }
                ],
            ),
            Node(
                package="path_planner",
                executable="planner_node",
                name="planner_node",
                output="screen",
                emulate_tty=True,
                parameters=[LaunchConfiguration("config_file"), planner_common_params],
            ),
            Node(
                package="simple_pid_controller",
                executable="simple_PID_controller_node",
                name="simple_PID_controller_node",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(LaunchConfiguration("dummy_control")),
                parameters=[LaunchConfiguration("config_file"), controller_common_params],
            ),
            Node(
                package="cpp_controller",
                executable="PID_controller_node",
                name="PID_controller_node_cpp",
                output="screen",
                emulate_tty=True,
                condition=UnlessCondition(LaunchConfiguration("dummy_control")),
                parameters=[LaunchConfiguration("config_file"), controller_common_params],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz_visualization")),
                arguments=["-d", os.path.join(package_share, "config", "rviz_config.rviz")],
                on_exit=Shutdown(),
            ),
        ]
    )
