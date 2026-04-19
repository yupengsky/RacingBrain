from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """整条感知链路启动骨架。

    TODO:
    - 声明相机话题、点云话题、标定文件、模型路径
    - 启动 cone_detector
    - 启动 test_cone_segmentation
    - 启动 fs_fusion_box
    """

    return LaunchDescription()
