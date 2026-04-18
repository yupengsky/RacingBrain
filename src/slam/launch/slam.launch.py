from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context):

    track = LaunchConfiguration("track").perform(context)
    rviz = LaunchConfiguration("rviz")

    pkg_share = get_package_share_directory("slam")

    base_params = os.path.join(pkg_share, "config", "params.yaml")
    track_params = os.path.join(pkg_share, "config", track + ".yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "slam.rviz")

    slam_node = Node(
        package="slam",
        executable="slam_node",
        name="slam_processor",
        output="screen",
        parameters=[base_params, track_params]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(rviz)   
    )

    return [
        slam_node,
        rviz_node
    ]


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            "track",
            default_value="acceleration",
            description="Track type: acceleration / autocross / skidpad"
        ),

        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz2"
        ),

        OpaqueFunction(function=launch_setup)

    ])