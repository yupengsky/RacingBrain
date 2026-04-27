import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    eval_debug = LaunchConfiguration('eval_debug')
    health_metrics = LaunchConfiguration('health_metrics')
    calibration_file = LaunchConfiguration('calibration_file')
    
    # 找到 yaml 文件的路径
    default_config_file = os.path.join(
        get_package_share_directory('fs_fusion_box'),
        'config',
        'calibration.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'eval_debug',
            default_value='false',
            description='Enable sidecar evaluation metrics publisher in fusion node.'
        ),
        DeclareLaunchArgument(
            'health_metrics',
            default_value='false',
            description='Enable lightweight runtime health metrics publisher in fusion node.'
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=default_config_file,
            description='Fusion calibration YAML file.'
        ),
        Node(
            package='fs_fusion_box',
            executable='fusion_box_node',
            name='fusion_box_node',
            output='screen',
            parameters=[
                calibration_file,
                {
                    'evaluation.enable_debug_metrics': ParameterValue(eval_debug, value_type=bool),
                    'runtime_health.enable_metrics': ParameterValue(health_metrics, value_type=bool),
                }
            ],
            
            # [关键修改]
            remappings=[
                # 雷达 (接 test_cone_segmentation)
                ('perception/lidar/cones_custom', '/cone_detection_custom'), 
                
                # 相机 (接 yolo_detector)
                # 左边是 cpp 里的名字，右边是 yolo_detector.py 发出的名字
                ('perception/camera/cones_custom', '/yolo/cones'),
                
                # 输出
                ('fusion/cones', '/perception/fusion/map')
            ]
        )
    ])
