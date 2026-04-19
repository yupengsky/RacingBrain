from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """SLAM 启动骨架。

    TODO:
    - 声明 track / rviz / export 参数
    - 加载 params.yaml + 赛道专属 yaml
    - 启动 slam_node
    - 可选启动点云导出节点 / RViz
    """

    return LaunchDescription()
