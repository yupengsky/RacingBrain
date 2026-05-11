import collections  # 用于创建双端队列
import math
import numpy as np

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from .vehicle_pid_controller import VehiclePIDController

from fs_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry
from drd25_msgs.msg import Path
from std_msgs.msg import Bool
from rcl_interfaces.msg import SetParametersResult


def euclidean_distance(v1, v2):  # 计算欧式距离
    return math.sqrt(sum([(a - b) ** 2 for a, b in zip(v1, v2)]))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def inertial_to_body_frame(ego_location, xi, yi, psi):
    """
    将点的坐标从全局坐标系表示转换到自车坐标系表示
    """
    Xi = np.array([xi, yi])  # 全局坐标系中的点
    R_psi_T = np.array([[np.cos(psi), np.sin(psi)],
                        [-np.sin(psi), np.cos(psi)]])  # 旋转矩阵
    Xt = np.array([ego_location[0], ego_location[1]])  # 全局坐标系中的自车位置
    Xb = np.matmul(R_psi_T, Xi - Xt)  # 转换到自车坐标系
    return Xb


class Controller(Node):
    def __init__(self, name):
        super().__init__(name)

        # 参数声明
        self.declare_parameter('control_time_step', 0.05)
        self.declare_parameter('target_speed', 15 / 3.6)
        self.declare_parameter('min_accepted_wp_num', 2)
        self.declare_parameter('recovery_max_steer', 0.35)  # 新增恢复模式参数
        self.declare_parameter('recovery_speed_ratio', 0.5)  # 新增恢复模式参数
        self.declare_parameter('odom_topic', '/testing_only/odom')
        self.declare_parameter('path_topic', '/drd25/path')
        self.declare_parameter('off_track_topic', '/drd25/off_track')
        self.declare_parameter('brake_topic', '/drd25/brake_command')
        self.declare_parameter('control_topic', '/control_command')

        # 参数初始化
        self.control_time_step = self.get_parameter('control_time_step').value
        self._target_speed = self.get_parameter('target_speed').value
        self.min_accepted_wp_num = self.get_parameter('min_accepted_wp_num').value
        self.recovery_max_steer = self.get_parameter('recovery_max_steer').value
        self.recovery_speed_ratio = self.get_parameter('recovery_speed_ratio').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.off_track_topic = self.get_parameter('off_track_topic').value
        self.brake_topic = self.get_parameter('brake_topic').value
        self.control_topic = self.get_parameter('control_topic').value

        # 状态变量
        self._current_pose = [0.0, 0.0, 0.0]
        self._current_speed = None
        self.recovery_mode = False
        buffer_size = 30
        self._waypoints_queue = collections.deque(maxlen=buffer_size)

        # 订阅者
        self._odometry_subscriber = self.create_subscription(Odometry, self.odom_topic, self.odometry_cb, 1,
                                                             callback_group=MutuallyExclusiveCallbackGroup())
        self._path_subscriber = self.create_subscription(Path, self.path_topic, self.path_cb, 1,
                                                         callback_group=MutuallyExclusiveCallbackGroup())
        # 新增恢复模式状态订阅
        self._offtrack_subscriber = self.create_subscription(Bool, self.off_track_topic, self.offtrack_cb, 1,
                                                            callback_group=MutuallyExclusiveCallbackGroup())
        self._brake_subscriber = self.create_subscription(Bool, self.brake_topic, self.brake_cb, 1)
        self._brake_flag = False  # 新增刹车标志
        # 发布者
        self._control_cmd_publisher = self.create_publisher(ControlCommand, self.control_topic, 1,
                                                            callback_group=MutuallyExclusiveCallbackGroup())
        # 控制器
        self._vehicle_controller = VehiclePIDController()
        self.handle_parameters()
        self.timer = None

        self.add_on_set_parameters_callback(self.parameters_cb)

    def brake_cb(self, msg):
        """刹车信号回调"""
        self._brake_flag = msg.data
        
    def offtrack_cb(self, msg):
        """恢复模式状态回调"""
        self.recovery_mode = msg.data
        if self.recovery_mode:
            # 立即降低目标速度
            base_speed = self.get_parameter('target_speed').value
            self._target_speed = min(3.0, base_speed * self.recovery_speed_ratio)

    def odometry_cb(self, odom_msg):
        pose = [0, 0, 0]
        pose[0] = odom_msg.pose.pose.position.x
        pose[1] = odom_msg.pose.pose.position.y
        self._current_speed = math.sqrt(odom_msg.twist.twist.linear.x ** 2 +
                                        odom_msg.twist.twist.linear.y ** 2)
        pose[2] = yaw_from_quaternion(odom_msg.pose.pose.orientation)
        self._current_pose = pose

    def path_cb(self, path_msg):
        if len(path_msg.waypoints) >= self.min_accepted_wp_num:
            ego_location = self._current_pose[:2]
            path_final_point = [path_msg.waypoints[-1].x, path_msg.waypoints[-1].y]
            front = inertial_to_body_frame(ego_location, path_final_point[0],
                                           path_final_point[1], self._current_pose[2])[0] >= 0.0
            if front:
                self._waypoints_queue.clear()
                self._waypoints_queue.extend([[wp.x, wp.y] for wp in path_msg.waypoints])
        if self.timer is None:
            self.timer = self.create_timer(self.control_time_step, self.run_step,
                                           callback_group=MutuallyExclusiveCallbackGroup())

    def run_step(self):
        if self._current_speed is None:
            return
        idx = self.closest_wp_idx(0)
        if idx >= len(self._waypoints_queue) - 1:
            return
        target_pose = self._waypoints_queue[idx]
        
        if self._brake_flag:  # 刹车优先
            cmd = ControlCommand()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.throttle = 0.0
            cmd.brake = 1.0  # 最大刹车
            cmd.steering = 0.0
            self._control_cmd_publisher.publish(cmd)
            return  # 跳过正常控制逻辑
        
        # 恢复模式速度控制
        base_speed = self.get_parameter('target_speed').value
        if self.recovery_mode:
            target_speed = min(2.5, base_speed * self.recovery_speed_ratio)
            max_steer = self.recovery_max_steer
        else:
            target_speed = base_speed
            max_steer = 1.0

        # 执行控制计算
        control_command = self._vehicle_controller.run_step(
            target_speed * 3.6,
            self._current_speed * 3.6,
            self._current_pose,
            target_pose
        )

        # 恢复模式转向限制
        if self.recovery_mode:
            steer = np.clip(control_command[0], -max_steer, max_steer)
            control_command = (steer, control_command[1])

        # 发布控制指令
        if control_command[0] is not None and control_command[1] is not None:
            cmd = ControlCommand()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.throttle = control_command[1] if control_command[1] > 0 else 0.0
            cmd.brake = -control_command[1] if control_command[1] < 0 else 0.0
            cmd.steering = -control_command[0]
            self._control_cmd_publisher.publish(cmd)

    # 以下原有方法保持不变
    def closest_wp_idx(self, f_idx, w_size=30):
        """
        给出自车全局状态和规划的局部路径，找到路径点f_idx前w_size内距离自车最近且在其前方的路径点
        """
        min_dist = float("inf")  # 保存最近的距离
        ego_location = self._current_pose[:2]  # 自车位置x，y
        closest_wp_index = 0  # default WP
        w_size = w_size if w_size <= len(self._waypoints_queue) - f_idx \
            else len(self._waypoints_queue) - f_idx  # 搜索点数范围
        for i in range(w_size):
            temp_wp = self._waypoints_queue[f_idx + i]
            temp_dist = euclidean_distance(ego_location, temp_wp)
            if temp_dist <= min_dist \
                    and inertial_to_body_frame(ego_location, temp_wp[0], temp_wp[1], self._current_pose[2])[0] > 0.0:
                closest_wp_index = i
                min_dist = temp_dist
        if min_dist < 1.0:
            closest_wp_index += 1

        return f_idx + closest_wp_index
    
    def handle_parameters(self):
        member_variables = self._vehicle_controller.cfg.__dict__.keys()
        for param in member_variables:
            self.declare_parameter(param, value=self._vehicle_controller.cfg.__dict__[param])
            self._vehicle_controller.cfg.__dict__[param] = self.get_parameter(param).value
            self._vehicle_controller._lon_controller.cfg.__dict__[param] = self.get_parameter(param).value
            self._vehicle_controller._lat_controller.cfg.__dict__[param] = self.get_parameter(param).value

    def parameters_cb(self, params):
        # 增加新参数处理
        for param in params:
            if param.name in self._vehicle_controller.cfg.__dict__.keys():
                old_value = self._vehicle_controller.cfg.__dict__[param.name]
                self._vehicle_controller.cfg.__dict__[param.name] = param.value
                self._vehicle_controller._lon_controller.cfg.__dict__[param.name] = param.value
                self._vehicle_controller._lat_controller.cfg.__dict__[param.name] = param.value
                print(f"{param.name} changed from {old_value} to {param.value}")
            elif param.name == 'control_time_step':
                old_value = self.control_time_step
                self.destroy_timer(self.timer)
                self.control_time_step = param.value
                self.timer = self.create_timer(self.control_time_step, self.run_step,
                                               callback_group=MutuallyExclusiveCallbackGroup())
                print(f"Control time step changed from {old_value} to {self.control_time_step}")
            elif param.name == 'target_speed':
                old_value = self._target_speed
                self._target_speed = param.value
                print(f"target_speed changed from {old_value} to {param.value}")
            elif param.name == 'min_accepted_wp_num':
                old_value = self.min_accepted_wp_num
                self.min_accepted_wp_num = param.value
                print(f"min_accepted_wp_num changed from {old_value} to {param.value}")
            elif param.name == 'recovery_max_steer':
                self.recovery_max_steer = param.value
                print(f"recovery_max_steer changed to {param.value}")
            elif param.name == 'recovery_speed_ratio':
                self.recovery_speed_ratio = param.value
                print(f"recovery_speed_ratio changed to {param.value}")
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = Controller("simple_PID_controller_node")
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
