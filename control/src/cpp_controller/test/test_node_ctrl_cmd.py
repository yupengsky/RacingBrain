#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from fs_msgs.msg import ControlCommand
import matplotlib.pyplot as plt
from collections import deque
import numpy as np

class ControlCommandPlotter(Node):
    def __init__(self):
        super().__init__('control_command_plotter')

        # 订阅控制指令
        self.control_sub = self.create_subscription(
            ControlCommand,
            '/control_command',
            self.control_callback,
            10
        )

        # 初始化数据存储（使用deque限制数据长度）
        self.time_data = deque(maxlen=1000)   # 时间戳
        self.throttle_data = deque(maxlen=1000)
        self.brake_data = deque(maxlen=1000)
        self.steering_data = deque(maxlen=1000)

        # 初始化绘图窗口
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(8, 10))
        self.fig.suptitle('Control Command Visualization')

        # 初始化曲线对象
        self.line_throttle, = self.ax1.plot([], [], 'r-', label='Throttle')
        self.line_brake, = self.ax2.plot([], [], 'g-', label='Brake')
        self.line_steering, = self.ax3.plot([], [], 'b-', label='Steering')

        # 配置子图属性
        self.configure_axes()

        # 标志位用于首次绘图初始化
        self.first_plot = True

    def configure_axes(self):
        for ax in [self.ax1, self.ax2, self.ax3]:
            ax.grid(True)
            ax.relim()
            ax.autoscale_view(True, True, True)

        self.ax1.set_ylabel('Throttle (%)')
        self.ax2.set_ylabel('Brake (%)')
        self.ax3.set_ylabel('Steering (rad)')
        self.ax3.set_xlabel('Time (s)')

        for ax in [self.ax1, self.ax2, self.ax3]:
            ax.legend(loc='upper right')

    def control_callback(self, msg):
        # 提取数据
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # 存储数据
        self.time_data.append(timestamp)
        self.throttle_data.append(msg.throttle)
        self.brake_data.append(msg.brake)
        self.steering_data.append(msg.steering)

        # 更新绘图
        self.update_plot()

    def update_plot(self):
        # 转换数据为numpy数组
        t = np.array(self.time_data)
        throttle = np.array(self.throttle_data)
        brake = np.array(self.brake_data)
        steering = np.array(self.steering_data)

        # 更新曲线数据
        self.line_throttle.set_data(t, throttle)
        self.line_brake.set_data(t, brake)
        self.line_steering.set_data(t, steering)

        # 调整坐标轴范围
        for ax in [self.ax1, self.ax2, self.ax3]:
            ax.relim()
            ax.autoscale_view(True, True, True)

        # 首次绘图需要完整绘制
        if self.first_plot:
            plt.show(block=False)
            self.first_plot = False

        # 强制刷新图形
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def destroy_node(self):
        # 关闭图形窗口
        plt.close(self.fig)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    plotter = ControlCommandPlotter()

    try:
        # 使用非阻塞spin
        while rclpy.ok():
            rclpy.spin_once(plotter, timeout_sec=0.1)
            plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
