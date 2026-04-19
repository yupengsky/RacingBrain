from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """融合节点启动骨架。

    TODO:
    - 声明相机与 LiDAR 输入 topic
    - 加载 calibration.yaml
    - 启动 fs_fusion_box_node
    """

    return LaunchDescription()
