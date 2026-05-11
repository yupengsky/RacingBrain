#!/usr/bin/env python3  

import rclpy  
from rclpy.node import Node  
from nav_msgs.msg import Odometry  
from drd25_msgs.msg import Path  
from std_msgs.msg import Bool  # 引入 Bool 消息类型  
import matplotlib.pyplot as plt  
import numpy as np  

class TrajectoryVisualizer(Node):  
    def __init__(self):  
        super().__init__('trajectory_visualizer')  

        # 订阅车辆实际轨迹 (Odometry)  
        self.odom_sub = self.create_subscription(  
            Odometry,  
            '/testing_only/odom',  # 车辆实际轨迹话题  
            self.odom_callback,  
            10  
        )  

        # 订阅预定轨迹 (Path)  
        self.path_sub = self.create_subscription(  
            Path,  
            '/drd25/path',  # 预定轨迹话题  
            self.path_callback,  
            10  
        )  

        # 新增订阅器：订阅 Bool 类型的消息  
        self.reset_sub = self.create_subscription(  
            Bool,  
            '/trajectory_visualizer/reset',  # Bool 消息的话题  
            self.reset_callback,  
            10  
        )  

        # 初始化绘图  
        self.fig, self.ax = plt.subplots(figsize=(8, 8))  # 创建绘图窗口  
        self.fig.suptitle('Vehicle Trajectory Tracking')  

        # 初始化数据存储  
        self.vehicle_x = []  # 车辆实际轨迹的 x 坐标  
        self.vehicle_y = []  # 车辆实际轨迹的 y 坐标  
        self.planned_x = []  # 预定轨迹的 x 坐标  
        self.planned_y = []  # 预定轨迹的 y 坐标  
        self.current_speed = 0.0  # 存储当前速度

        # 设置图形属性  
        self.ax.set_xlabel('X Position')  
        self.ax.set_ylabel('Y Position')  
        self.ax.grid(True)  

        # 用于存储横向偏差  
        self.lateral_errors = []  

        # 标志位，表示是否结束  
        self.finished = False  

    def odom_callback(self, msg):  
        # 获取车辆实际位置  
        x = msg.pose.pose.position.x  
        y = msg.pose.pose.position.y  

        # 存储车辆实际位置  
        self.vehicle_x.append(x)  
        self.vehicle_y.append(y)  

        # 获取并存储当前速度  
        self.current_speed = np.sqrt(msg.twist.twist.linear.x ** 2 + msg.twist.twist.linear.y ** 2)  

        # 计算横向偏差（假设预定路径是直线或已知路径）  
        if self.planned_x and self.planned_y:  
            # 找到最近的预定路径点  
            distances = np.sqrt((np.array(self.planned_x) - x) ** 2 + (np.array(self.planned_y) - y) ** 2)  
            closest_idx = np.argmin(distances)  
            closest_x = self.planned_x[closest_idx]  
            closest_y = self.planned_y[closest_idx]  

            # 计算横向偏差（垂直于路径方向）  
            if closest_idx > 0:  # 确保 closest_idx - 1 不越界  
                lateral_error = np.abs((self.planned_y[closest_idx] - self.planned_y[closest_idx - 1]) * x -  
                                      (self.planned_x[closest_idx] - self.planned_x[closest_idx - 1]) * y +  
                                      self.planned_x[closest_idx] * self.planned_y[closest_idx - 1] -  
                                      self.planned_y[closest_idx] * self.planned_x[closest_idx - 1]) / \
                                np.sqrt((self.planned_y[closest_idx] - self.planned_y[closest_idx - 1]) ** 2 +  
                                        (self.planned_x[closest_idx] - self.planned_x[closest_idx - 1]) ** 2)  
                self.lateral_errors.append(lateral_error)  

        # 更新绘图  
        self.update_plot()  

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
            self.lateral_errors.clear()  
            # 刷新图形  
            plt.pause(0.01)  

    def update_plot(self):  
        # 绘制预定轨迹  
        if self.planned_x and self.planned_y:  
            self.ax.plot(self.planned_x, self.planned_y, color='blue', linestyle='--', linewidth=2)  

        # 绘制车辆实际轨迹  
        if self.vehicle_x and self.vehicle_y:  
            self.ax.plot(self.vehicle_x, self.vehicle_y, color='orange', linewidth=2)  

        # 显示当前速度  
        #self.ax.set_title(f'Vehicle Trajectory Tracking\nCurrent Speed: {self.current_speed:.2f} m/s')  # 在标题中显示速度 

        speed_text = f'Current Speed: {self.current_speed:.2f} m/s'

        # 如果已有文本对象，则移除它
        if hasattr(self, 'speed_text_obj'):
            self.speed_text_obj.remove()
        
        # 创建并保存新的文本对象
        self.speed_text_obj = self.ax.text(0.5, -0.1, speed_text, transform=self.ax.transAxes,
                        fontsize=12, verticalalignment='top',
                        horizontalalignment='center',
                        bbox=dict(facecolor='yellow', alpha=0.3))

        # 刷新图形  
        plt.pause(0.01)  

    def calculate_lateral_errors(self):  
        # 计算三个最大的横向偏差  
        if self.lateral_errors:  
            sorted_errors = sorted(self.lateral_errors, reverse=True)  
            top_3_errors = sorted_errors[:3]  
            formatted_errors = [round(error, 2) for error in top_3_errors]  
            print("Top 3 Lateral Errors (Max to Min):", formatted_errors)  
        else:  
            print("No lateral errors recorded.")  

    def calculate_variance(self):  
        # 计算实际路径和规定路径的方差  
        if self.vehicle_x and self.planned_x:  
            min_length = min(len(self.vehicle_x), len(self.planned_x))  
            vehicle_x = self.vehicle_x[:min_length]  
            vehicle_y = self.vehicle_y[:min_length]  
            planned_x = self.planned_x[:min_length]  
            planned_y = self.planned_y[:min_length]  

            variance_x = np.var(np.array(vehicle_x) - np.array(planned_x))  
            variance_y = np.var(np.array(vehicle_y) - np.array(planned_y))  
            print(f"Variance between Actual and Planned Path (X): {round(variance_x, 2)}")  
            print(f"Variance between Actual and Planned Path (Y): {round(variance_y, 2)}")  
        else:  
            print("Insufficient data to calculate variance.")  

    def destroy_node(self):  
        # 在节点销毁时输出结果  
        if not self.finished:  
            self.calculate_lateral_errors()  
            self.calculate_variance()  
            self.finished = True  
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