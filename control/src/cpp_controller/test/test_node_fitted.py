#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from drd25_msgs.msg import Path
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
import numpy as np

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')

        # 订阅预定轨迹 (Path)
        self.path_sub = self.create_subscription(
            Path,
            '/drd25/path',
            self.path_callback,
            10
        )

        # 订阅拟合后的路径
        self.fitted_path_sub = self.create_subscription(
            Path,
            '/drd25/fitted_path',
            self.fitted_path_callback,
            10
        )

        # 新增订阅器：订阅 Bool 类型的消息
        self.reset_sub = self.create_subscription(
            Bool,
            '/trajectory_visualizer/reset',
            self.reset_callback,
            10
        )

        # 初始化绘图
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.suptitle('Vehicle Trajectory Tracking')

        # 初始化数据存储
        self.vehicle_x = []  # 拟合后的轨迹的 x 坐标
        self.vehicle_y = []  # 拟合后的轨迹的 y 坐标
        self.planned_x = []  # 预定轨迹的 x 坐标
        self.planned_y = []  # 预定轨迹的 y 坐标

        # 设置图形属性
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.grid(True)

        # 标志位，表示是否结束
        self.finished = False

    def path_callback(self, msg):
        # 清空之前的预定轨迹数据
        self.planned_x.clear()
        self.planned_y.clear()

        # 提取预定轨迹的路径点
        for waypoint in msg.waypoints:
            x = waypoint.x  # 假设 Waypoint 消息中有 x 和 y 字段
            y = waypoint.y
            self.planned_x.append(x)
            self.planned_y.append(y)

        # 更新绘图
        self.update_plot()

    def fitted_path_callback(self, msg):
        # 清空之前的拟合轨迹数据
        self.vehicle_x.clear()
        self.vehicle_y.clear()

        # 提取拟合后的轨迹的路径点
        for waypoint in msg.waypoints:
            x = waypoint.x  # 假设 Waypoint 消息中有 x 和 y 字段
            y = waypoint.y
            self.vehicle_x.append(x)
            self.vehicle_y.append(y)

        # 更新绘图
        self.update_plot()

    def reset_callback(self, msg):
        # 如果收到 True，清空绘图窗口并重新开始绘制
        if msg.data:
            # 清空绘图窗口
            self.ax.clear()
            # 重置图形属性
            self.ax.set_xlabel('X Position')
            self.ax.set_ylabel('Y Position')
            self.ax.grid(True)
            # 清空数据存储
            self.vehicle_x.clear()
            self.vehicle_y.clear()
            self.planned_x.clear()
            self.planned_y.clear()
            # 刷新图形
            plt.pause(0.01)

    def update_plot(self):
        # 绘制预定轨迹
        if self.planned_x and self.planned_y:
            self.ax.plot(self.planned_x, self.planned_y, color='blue', linestyle='--', linewidth=2)

        # 绘制拟合后的轨迹
        if self.vehicle_x and self.vehicle_y:
            self.ax.plot(self.vehicle_x, self.vehicle_y, color='red', linewidth=2)

        # 添加图例
        self.ax.legend()

        # 刷新图形
        plt.pause(0.01)

    def destroy_node(self):
        # 在节点销毁时清理资源
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    trajectory_visualizer = TrajectoryVisualizer()

    try:
        rclpy.spin(trajectory_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        trajectory_visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
