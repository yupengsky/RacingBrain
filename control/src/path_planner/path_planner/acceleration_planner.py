import numpy as np
import math
from threading import Thread
from std_msgs.msg import Bool
from drd25_msgs.msg import Cone
from .visualization import publish_savedwaypoints_markers


class PlannerCFG:
    def __init__(self):
        # 锥桶检测参数
        self.min_cones_per_side = 3  # 每侧最少锥桶数
        self.frontConesDist = 15.0  # 前方锥桶搜索范围（米）
        self.frontAngle = 90.0  # 前方锥桶搜索角度（度），90表示180度半圆范围
        self.behindDist = 0.5  # 车辆后方距离
        
        self.maxDistToSaveWaypoints = 5.0  # 最大保存路径点距离
        self.maxWaypointAmountToSave = 3  # 最大保存路径点数量
        
        # 路径生成参数
        self.lookahead_distance = 15  # 前瞻距离
        self.path_points_count = 15  # 路径点数量
        
        # 赛道参数
        self.track_length = 200  # 赛道总长度
        self.max_lateral_deviation = 2.5  # 最大横向偏差


class AccelerationPlanner:
    def __init__(self, node):
        self.rviz_visualization = node.rviz_visualization
        self.cfg = PlannerCFG()
        self.node = node
        
        # 状态变量
        self.cones = []  # 所有锥桶 (包含颜色信息)
        self.egopose = np.array([0.0, 0.0, 0.0])  # [x, y, yaw]
        self.saved_line_points = []  # 保存的路径点

        # 赛道边界
        self.left_slope = 0.0  # 左侧边界斜率
        self.right_slope = 0.0  # 右侧边界斜率
        self.center_slope = 0.0  # 中心线斜率
        self.left_intercept = 0.0  # 左侧截距
        self.right_intercept = 0.0  # 右侧截距
        
        # 行驶状态
        self.start_pose = None  # 起始位置记录
        self.distance_traveled = 0.0  # 已行驶距离
        self.recovery_mode = False  # 恢复模式

    # 更新锥桶地图
    def update_map(self, cones):
        if len(cones) >= 2:
            self.cones = cones

    # 更新车辆位姿
    def update_egopose(self, egopose):
        if self.start_pose is None:  # 记录初始位置
            self.start_pose = egopose.copy()
        else:  # 计算累计行驶距离
            delta = np.linalg.norm(egopose[:2] - self.start_pose[:2])
            self.distance_traveled += delta
            self.start_pose = egopose.copy()  # 更新为最新位置
        self.egopose = egopose
    
    # 主规划函数
    def plan(self):
        if len(self.cones) == 0:
            return None
        
         # 直线赛道刹车判断
        if self.distance_traveled >= self.cfg.track_length:  # 超过赛道长度
            brake_msg = Bool()
            brake_msg.data = True
            self.node.brake_pub.publish(brake_msg)
            return None  # 停止路径规划

        # 分离前方半圆范围内的左右锥桶（根据颜色）
        left_cones, right_cones = self._get_front_cones_by_color()
        
        # 检查锥桶数量
        if len(left_cones) < self.cfg.min_cones_per_side or len(right_cones) < self.cfg.min_cones_per_side:
            path = self._get_fallback_path()
            if path:
                self.fit_line(path)  # 筛选并可视化路径点
            return path
        
        print(f"使用前方锥桶拟合: 左{len(left_cones)} 右{len(right_cones)}")
        
        # 拟合左右边界直线
        left_success = self._fit_simple_line(left_cones, is_left=True)
        right_success = self._fit_simple_line(right_cones, is_left=False)
        
        if not (left_success and right_success):
            path = self._get_fallback_path()
            if path:
                self.fit_line(path)  # 筛选并可视化路径点
            return path
        
        # 计算中心线（平均斜率）
        self.center_slope = (self.left_slope + self.right_slope) / 2
        print(f"中心线: 斜率={self.center_slope:.3f}")

        # 检查是否异常
        if self._check_abnormal():
            self.recovery_mode = True
            path = self._get_recovery_path()
        else:
            self.recovery_mode = False
            path = self._get_centerline_path()
        
        if path:
            self.fit_line(path)  # 筛选并可视化路径点
        
        return path

    def _get_front_cones_by_color(self):
        """获取车辆前方半圆范围内的锥桶，根据颜色判断左右侧"""
        left_cones = []  # 前方左侧锥桶
        right_cones = []  # 前方右侧锥桶
        cone_schema = Cone()
        blue_value = getattr(cone_schema, 'BLUE', 0)
        yellow_value = getattr(cone_schema, 'YELLOW', getattr(cone_schema, 'RED', 1))
        right_color_values = {
            yellow_value,
            getattr(cone_schema, 'YELLOW_BIG', yellow_value),
            getattr(cone_schema, 'YELLOW_SMALL', yellow_value),
            getattr(cone_schema, 'ORANGE_BIG', yellow_value),
            getattr(cone_schema, 'ORANGE_SMALL', yellow_value),
        }
        
        # 获取车辆位置和航向
        vehicle_pos = self.egopose[:2]
        heading = self._get_heading_vector()
        heading_yaw = np.arctan2(heading[1], heading[0])
        
        for cone in self.cones:
            # 提取锥桶坐标
            cone_x, cone_y = cone.x, cone.y
            cone_pos = np.array([cone_x, cone_y])
            
            # 计算相对向量和距离
            vec = cone_pos - vehicle_pos
            dist = np.linalg.norm(vec)
            
            # 距离过滤：太远或太近忽略
            if dist > self.cfg.frontConesDist or dist < 0.5:
                continue
            
            # 计算相对角度（相对于车辆航向）
            cone_angle = np.arctan2(vec[1], vec[0])
            angle_diff = self._wrap_angle(cone_angle - heading_yaw)
            angle_diff_deg = np.abs(angle_diff) * 180.0 / np.pi
            
            # 角度过滤：只保留前方半圆范围内
            if angle_diff_deg > self.cfg.frontAngle:
                continue
            
            # 根据颜色判断左右侧
            if hasattr(cone, 'color'):
                if cone.color == blue_value:  # 蓝色=左侧
                    left_cones.append(cone_pos)
                elif cone.color in right_color_values:  # 黄色/红色/橙色=右侧或特殊标志桶
                    right_cones.append(cone_pos)
                else:
                    # 未知颜色，根据位置判断
                    cross_product = np.cross(heading, vec)
                    if cross_product > 0:
                        left_cones.append(cone_pos)
                    else:
                        right_cones.append(cone_pos)
            else:
                # 没有颜色属性，根据位置判断
                cross_product = np.cross(heading, vec)
                if cross_product > 0:
                    left_cones.append(cone_pos)
                else:
                    right_cones.append(cone_pos)
        
        return np.array(left_cones), np.array(right_cones)
    
    def _wrap_angle(self, angle):
        """角度归一化到[-π, π]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    # 简单线性拟合（最小二乘法）
    def _fit_simple_line(self, cones, is_left):
        if len(cones) < 2:
            return False
        
        x = cones[:, 0]
        y = cones[:, 1]
        
        # 简单最小二乘拟合：y = kx + b
        n = len(x)
        sum_x = np.sum(x)
        sum_y = np.sum(y)
        sum_xy = np.sum(x * y)
        sum_x2 = np.sum(x * x)
        
        # 计算斜率和截距
        denominator = n * sum_x2 - sum_x * sum_x
        if abs(denominator) < 1e-6:  # 避免除零
            return False
        
        k = (n * sum_xy - sum_x * sum_y) / denominator
        b = (sum_y * sum_x2 - sum_x * sum_xy) / denominator
        
        # 保存结果
        if is_left:
            self.left_slope = k
            self.left_intercept = b
            print(f"左侧边界: 斜率={k:.3f}, 截距={b:.3f}")
        else:
            self.right_slope = k
            self.right_intercept = b
            print(f"右侧边界: 斜率={k:.3f}, 截距={b:.3f}")
        
        return True
    
    # 生成沿中心线的路径
    def _get_centerline_path(self):
        if self.center_slope == 0 and hasattr(self, 'left_slope') and hasattr(self, 'right_slope'):
            self.center_slope = (self.left_slope + self.right_slope) / 2
        # 计算方向角度
        angle = np.arctan(self.center_slope)
        
        # 创建方向向量
        direction = np.array([np.cos(angle), np.sin(angle)])
        
        # 确保方向向前
        heading = self._get_heading_vector()
        if np.dot(direction, heading) < 0:
            direction = -direction
        
        # 生成路径点
        path = []
        for i in range(self.cfg.path_points_count):
            dist = i * (self.cfg.lookahead_distance / self.cfg.path_points_count)
            point = self.egopose[:2] + direction * dist
            path.append(point)
        
        return path
    
    # 备选路径：沿当前方向直行
    def _get_fallback_path(self):
        direction = self._get_heading_vector()
        
        path = []
        for i in range(self.cfg.path_points_count):
            dist = i * 1.0  # 1米间隔
            point = self.egopose[:2] + direction * dist
            path.append(point)
        
        return path

    # 恢复路径：保守前进
    def _get_recovery_path(self):
        direction = self._get_heading_vector()
        
        path = []
        for i in range(8):  # 生成8个点
            dist = (i + 1) * 2.0  # 2米间隔
            point = self.egopose[:2] + direction * dist
            path.append(point)
        
        return path

    # 检查异常情况
    def _check_abnormal(self):
        # 检查斜率差异
        if abs(self.left_slope - self.right_slope) > 2.0:  # 斜率差异太大
            return True
        
        # 检查横向偏差
        lateral_dev = self._get_lateral_deviation()
        if lateral_dev > self.cfg.max_lateral_deviation:
            return True
        
        return False

    # 计算与规划路径的横向偏差
    def _get_lateral_deviation(self):
        if not hasattr(self, 'center_slope') or self.center_slope == 0:
            return 0
        
        # 计算当前位置到中心线的垂直距离
        x, y = self.egopose[:2]
        center_y = self.center_slope * x  # 假设截距为0，简化计算
        
        angle = np.arctan(self.center_slope)
        deviation = (y - center_y) * np.cos(angle)
        
        return abs(deviation)
    
    # 获取车辆前进方向
    def _get_heading_vector(self):
        yaw = self.egopose[2]
        return np.array([np.cos(yaw), np.sin(yaw)])
    
    @staticmethod
    def dist(x1, y1, x2, y2):
        distSq = (x1 - x2) ** 2 + (y1 - y2) ** 2
        return math.sqrt(distSq)

    # 路径点筛选与可视化
    def fit_line(self, waypoints):
        if not waypoints:
            return
        newSavedPoints = []
        for i in range(len(waypoints)):
            waypointCandidate = waypoints[i]
            carWaypointDist = self.dist(self.egopose[0], self.egopose[1], waypointCandidate[0], waypointCandidate[1])
            
            # 每几个点取1个，减少密度
            if i % 15 != 0: 
                continue

            if i >= self.cfg.maxWaypointAmountToSave or carWaypointDist > self.cfg.maxDistToSaveWaypoints:
                break
            else:
                newSavedPoints.append(waypointCandidate)
                self.saved_line_points.append(waypointCandidate)
        
        # 限制保存的路径点数量，防止内存泄漏
        max_saved_points = 50
        if len(self.saved_line_points) > max_saved_points:
            self.saved_line_points = self.saved_line_points[-max_saved_points:]
        
        if self.rviz_visualization:
            savedwaypoints_markers_thread = Thread(target=publish_savedwaypoints_markers,
                                                   args=(self.node, self.saved_line_points))
            savedwaypoints_markers_thread.daemon = True
            savedwaypoints_markers_thread.start()
