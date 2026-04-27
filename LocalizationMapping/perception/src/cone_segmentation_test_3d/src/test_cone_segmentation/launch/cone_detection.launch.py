#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的路径
    package_dir = get_package_share_directory('test_cone_segmentation')
    
    # 参数文件路径
    config_file = os.path.join(package_dir, 'config', 'cone_detection_params.yaml')
    
    # 声明launch参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug output'
    )
    
    # 锥筒检测节点
    cone_detection_node = Node(
        package='test_cone_segmentation',
        executable='cone_segmentation_node',
        name='cone_segmentation_node',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        debug_arg,
        cone_detection_node
    ])
