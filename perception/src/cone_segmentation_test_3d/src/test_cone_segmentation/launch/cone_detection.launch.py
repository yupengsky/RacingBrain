from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """LiDAR 锥桶分割启动骨架。

    TODO:
    - 加载点云输入 topic
    - 加载分割参数 yaml
    - 启动 test_cone_segmentation 节点
    """

    return LaunchDescription()
