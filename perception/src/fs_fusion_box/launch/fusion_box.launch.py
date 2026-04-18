import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 找到 yaml 文件的路径
    config_file = os.path.join(
        get_package_share_directory('fs_fusion_box'),
        'config',
        'calibration.yaml'
    )

    return LaunchDescription([
        Node(
            package='fs_fusion_box',
            executable='fusion_box_node',
            name='fusion_box_node',
            output='screen',
            parameters=[config_file], 
            
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