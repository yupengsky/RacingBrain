
import numpy as np
import math
from enum import Enum
from threading import Thread
from scipy.spatial import Delaunay
from drd25_msgs.msg import Cone
from std_msgs.msg import Bool
from .RRT import RRT, Node
from .circle_fit import ransac_circle
from .visualization import (publish_tree_markers, publish_bestbranch_markers, publish_savedwaypoints_markers,
                            publish_delaunay_markers, publish_circle_markers)


class PlannerCFG:
    def __init__(self):
        # RRT
        self.frontConesDist = 15.0  # 桩筒搜索范围
        self.coneObstacleSize = 1.1  # 桩筒障碍物大小
        self.coneTargetsDistRatio = 0.5  # 目标桩筒范围比例
        self.iterationNumber = 150  # 最大迭代次数
        self.planDistance = 15.0  # 目标规划距离
        self.frontConesBiggerDist = 20.0  # 桩筒前视范围
        self.behindDist = 0.5  # 车辆后方距离
        self.expandDistance = 1.0  # 树枝扩展距离
        self.expandAngle = 20.0  # 树枝扩展角度
        self.maxTargetAroundDist = 3.0  # 目标桩筒周围采样范围
        self.nearSearchRatio = 3.0  # RRT重选父节点与重连的搜索范围与扩展距离的比例
        # 左右桩筒判断
        self.nearRatio = 0.4  # 桩筒距离比例
        self.maxAngleDiff = 30.0  # 最大角度差
        self.middle_track_half_width = 1.8  # 赛道出入口半宽度
        # 评分函数
        self.coneDistLimit = 4.0  # 桩筒距离限制(一般可设置为赛道宽度)
        self.correctColorReward = 1.0  # 颜色正确奖励
        self.yawChangepenaltyFactor = 1.0  # 角度变化惩罚
        # 平滑路径
        self.everyPointDistChangeLimit = 2.0  # 每个路径点的距离变化限制
        self.newPointFilter = 0.2  # 新路径点的滤波因子
        self.resetCount = 2  # 重置计数
        # Delaunay
        self.maxAcceptedEdgeLength = 6.0  # 最大接受边长
        self.maxEdgePartsRatio = 3.0  # 交点分割边长最大比例
        self.maxSideDiffFactor = 0.8  # 左右桩筒数量差异系数
        self.maxwpAngleDiff = 50.0  # 路径点角度差异
        # 合并路径
        self.maxDistToSaveWaypoints = 2.0  # 最大保存路径点距离
        self.maxWaypointAmountToSave = 2  # 最大保存路径点数量
        # 8字相关
        self.middle_cones_max_distance = 20.0  # 中心桩筒距离自车起始位置的最大距离self.delay_count = 30
        self.travel_distance_before_start = 4.0  # 车辆行驶一定距离后开始计算赛道中心点(防止赛道起点桩筒的干扰)
        self.distance_before_start = 1.5  # 从开始状态进入第一个右圈时，车辆距离赛道中心点的距离
        self.distance_first_right_to_second_right = 1.3  # 从第一个右圈进入第二个右圈时，车辆距离赛道中心点的距离
        self.distance_second_right_to_first_left = 2.5  # 从第二个右圈进入第一个左圈时，车辆距离赛道中心点的距离
        self.distance_first_left_to_second_left = 1.3  # 从第一个左圈进入第二个左圈时，车辆距离赛道中心点的距离
        self.distance_ending = 1.3  # 从第二个左圈结束到终点时，车辆距离赛道中心点的距离
        self.forward_guidance_dist = 15  # 在开始和结束阶段的前视引导距离
        self.delay_count = 30  # 第二个右圈开始后延迟多少个规划步，再开始判断是否进入第一个左圈
        self.ending_count = 10  # 结束阶段规划次数
        # 圆拟合
        self.target_radius = 8.2  # 目标半径
        self.saved_circle_points_num_before_fitting = 40  # 拟合前保存的路径点数量
        self.fit_num_iterations = 5  # 拟合迭代次数
        self.fit_error_threshold = 0.5  # 拟合误差阈值
        self.fit_n_pts = 30  # 拟合随机选择的点数量
        self.total_fitted_circle_points = 60  # 拟合后保存的路径点数量
        self.radius_change_factor = 0.2  # 半径变化因子
        self.pub_wp_num = 20  # 发布的路径点数量

'''
class SkidPadStage(Enum):
    BEFORE_START = 0
    FIRST_RIGHT_LOOP = 1
    SECOND_RIGHT_LOOP = 2
    FIRST_LEFT_LOOP = 3
    SECOND_LEFT_LOOP = 4
    ENDING = 5
'''
class SkidPadStage(Enum):
    BEFORE_START = 0
    FIRST_RIGHT_LOOP = 1
    FIRST_LEFT_LOOP = 2
    ENDING = 3

class ConeColor(Enum):
    BLUE = getattr(Cone(), 'BLUE', 0)
    YELLOW = getattr(Cone(), 'YELLOW', getattr(Cone(), 'RED', 1))
    ORANGE_BIG = getattr(Cone(), 'ORANGE_BIG', getattr(Cone(), 'YELLOW_BIG', 2))
    ORANGE_SMALL = getattr(Cone(), 'ORANGE_SMALL', getattr(Cone(), 'YELLOW_SMALL', 3))
    UNKNOWN = getattr(Cone(), 'UNKNOWN', 4)


class SkidPadPlanner:
    def __init__(self, node):
        self.rviz_visualization = node.rviz_visualization
        self.cfg = PlannerCFG()
        self.node = node
        self.cones = []  # 桩筒坐标
        self.egopose = np.array([0.0, 0.0, 0.0])  # 车辆位置(x, y, yaw)
        self.start_pose = None
        self.stage = SkidPadStage.BEFORE_START
        self.track_center = None  # 赛道中心点（计时器位置）
        self.forward_vector = None  # 车辆前进方向
        self.filteredBestBranch = []
        self.circle = None  # x, y, r
        self.saved_circle_points = []
        self.guessed_left_circle_center = None
        self.guessed_right_circle_center = None
        self.last_radius = None
        self.idx = None
        self.count = 0
        self.ending_start_time = None  # 新增ENDING阶段开始时间
        self.recovery_mode = False

    def update_map(self, cones):
        if len(cones) >= 2:
            self.cones = cones

    def update_egopose(self, egopose):
        if not self.egopose.any():
            self.start_pose = egopose
        self.egopose = egopose

    def plan(self):
        if self.stage == SkidPadStage.BEFORE_START:
            if len(self.cones) == 0:
                return None
            frontcones_all, frontconescolor_all = self.get_frontcones_with_color(self.cfg.frontConesDist)
            # 寻找赛道中心点
            middle_four_cones_mask = np.array(frontconescolor_all) == ConeColor.ORANGE_BIG.value
            middle_four_cones = np.array(frontcones_all)[middle_four_cones_mask]
            travel_distance = np.linalg.norm(self.egopose[:2] - self.start_pose[:2])
            if len(middle_four_cones) == 4 and travel_distance > self.cfg.travel_distance_before_start:
                # 当能感知到中心全部四个桩筒且车辆行驶一定距离后才计算赛道中心点
                self.track_center = np.mean(middle_four_cones, axis=0)
                right_vector = np.array([self.forward_vector[1], -self.forward_vector[0]])
                self.guessed_right_circle_center = self.track_center + 9.125 * right_vector
                self.guessed_left_circle_center = self.track_center - 9.125 * right_vector
                if np.linalg.norm(self.track_center - self.egopose[:2]) < self.cfg.distance_before_start:
                    print('FIRST_RIGHT_LOOP!')
                    self.stage = SkidPadStage.FIRST_RIGHT_LOOP
            heading_vector = self.getHeadingVector()
            forward_guidance_np = np.array([self.egopose[:2] + heading_vector * dis
                                            for dis in range(0, self.cfg.forward_guidance_dist)])
            forward_guidance = [Node(point[0], point[1], 0) for point in forward_guidance_np]
            if self.rviz_visualization:
                bestbranch_markers_thread = Thread(target=publish_bestbranch_markers,
                                                   args=(self.node, forward_guidance, 1))
                bestbranch_markers_thread.daemon = True
                bestbranch_markers_thread.start()

            frontcones = []
            for cone in np.array(frontcones_all):
                if abs(cone[1]) < self.cfg.middle_track_half_width:
                    frontcones.append(cone)
            delaunayEdges = self.getDelaunayEdges(frontcones)
            if delaunayEdges:
                if self.rviz_visualization:
                    delaunay_edges_thread = Thread(target=publish_delaunay_markers, args=(self.node, delaunayEdges))
                    delaunay_edges_thread.daemon = True
                    delaunay_edges_thread.start()
                newWaypoints = self.getWaypointsFromEdges(forward_guidance, delaunayEdges)
                if newWaypoints:
                    if self.forward_vector is None:
                        self.forward_vector = newWaypoints[-1] - newWaypoints[0]
                        self.forward_vector = self.forward_vector / np.linalg.norm(self.forward_vector)
                    return newWaypoints
        
        
        elif self.stage == SkidPadStage.FIRST_RIGHT_LOOP:
            if len(self.cones) == 0:
                return None
            if (self.circle is not None and np.linalg.norm(self.track_center - self.egopose[:2]) <
                self.cfg.distance_first_right_to_second_right):
                self.count = 0
                self.circle = None
                print('FIRST_LEFT_LOOP!')
                self.stage = SkidPadStage.FIRST_LEFT_LOOP
                self.saved_circle_points = []
                self.last_radius = None
            newWaypoints = self.plan_for_first_loop('right')
            if newWaypoints:
                WaypointstoPublish = self.fit_circle(newWaypoints, 'right')
                return WaypointstoPublish
 
        elif self.stage == SkidPadStage.FIRST_LEFT_LOOP:
            if len(self.cones) == 0:
                return None
            if (self.circle is not None and np.linalg.norm(self.track_center - self.egopose[:2])
                    < self.cfg.distance_ending):
                self.count = 0
                self.circle = None
                print('ENDING!')
                self.stage = SkidPadStage.ENDING
                self.saved_circle_points = []
                self.last_radius = None
                self.idx = 0
                return None
            newWaypoints = self.plan_for_first_loop('left')
            if newWaypoints:
                WaypointstoPublish = self.fit_circle(newWaypoints, 'left')
                return WaypointstoPublish

        elif self.stage == SkidPadStage.ENDING:
            if self.ending_start_time is None:  # 记录进入ENDING的时间
                self.ending_start_time = self.node.get_clock().now()
            
            # 检查0.5秒延迟
            elapsed_time = (self.node.get_clock().now() - self.ending_start_time).nanoseconds / 1e9
            if elapsed_time >= 5:
                brake_msg = Bool()
                brake_msg.data = True
                self.node.brake_pub.publish(brake_msg)
                return None  # 停止路径规划
            if len(self.cones) == 0:
                return None
            if self.idx >= self.cfg.ending_count:
                return None
            frontcones_all, frontconescolor_all = self.get_frontcones_with_color(self.cfg.frontConesDist)
            color_mask = np.array(frontconescolor_all) != ConeColor.ORANGE_BIG.value
            frontcones = []
            for cone in np.array(frontcones_all)[color_mask]:
                if abs(cone[1]) < self.cfg.middle_track_half_width:
                    frontcones.append(cone)
            forward_guidance_np = np.array([self.egopose[:2] + self.forward_vector * dis
                                            for dis in range(0, self.cfg.forward_guidance_dist)])
            forward_guidance = [Node(point[0], point[1], 0) for point in forward_guidance_np]
            if self.rviz_visualization:
                bestbranch_markers_thread = Thread(target=publish_bestbranch_markers,
                                                   args=(self.node, forward_guidance, 1))
                bestbranch_markers_thread.daemon = True
                bestbranch_markers_thread.start()
            delaunayEdges = self.getDelaunayEdges(frontcones)
            if delaunayEdges:
                if self.rviz_visualization:
                    delaunay_edges_thread = Thread(target=publish_delaunay_markers, args=(self.node, delaunayEdges))
                    delaunay_edges_thread.daemon = True
                    delaunay_edges_thread.start()
                newWaypoints = self.getWaypointsFromEdges(forward_guidance, delaunayEdges)
                if newWaypoints:
                    self.idx += 1
                    return newWaypoints

    def plan_for_first_loop(self, direction):
        newWaypoints = None
        frontcones_all, frontconescolor_all = self.get_frontcones_with_color(self.cfg.frontConesDist)
        color_mask = np.array(frontconescolor_all) != ConeColor.ORANGE_BIG.value
        frontcones = np.array(frontcones_all)[color_mask]
        coneObstacleList = []
        rrtConeTargets = []  # rrt目标
        for cone in frontcones:
            target = (cone[0], cone[1], self.cfg.coneObstacleSize)
            coneObstacleList.append(target)

            coneDist = np.linalg.norm(cone - self.egopose[:2])
            if coneDist > self.cfg.frontConesDist * self.cfg.coneTargetsDistRatio:
                rrtConeTargets.append(target)
        # Set Initial parameters
        start = self.egopose
        rrt = RRT(start, self.cfg, obstacleList=coneObstacleList, rrtTargets=rrtConeTargets)
        nodeList, leafNodes = rrt.Planning()
        if self.rviz_visualization:
            tree_markers_thread = Thread(target=publish_tree_markers, args=(self.node, nodeList, leafNodes))
            tree_markers_thread.daemon = True
            tree_markers_thread.start()
        largerGroupfrontcones_all, largerGroupfrontconescolor_all = (
            self.get_frontcones_with_color(self.cfg.frontConesBiggerDist))
        color_mask = np.array(largerGroupfrontconescolor_all) != ConeColor.ORANGE_BIG.value
        largerGroupfrontcones = np.array(largerGroupfrontcones_all)[color_mask]
        largerGroupfrontconescolor = np.array(largerGroupfrontconescolor_all)[color_mask]
        # 寻找最佳路径
        bestBranch = self.findBestBranch(leafNodes, nodeList, largerGroupfrontcones,
                                         largerGroupfrontconescolor, direction)
        if bestBranch:
            if self.rviz_visualization:
                bestbranch_markers_thread = Thread(target=publish_bestbranch_markers,
                                                   args=(self.node, bestBranch, 0))
                bestbranch_markers_thread.daemon = True
                bestbranch_markers_thread.start()
            filteredBestBranch = self.getFilteredBestBranch(bestBranch)
            if filteredBestBranch:
                if self.rviz_visualization:
                    bestbranch_markers_thread = Thread(target=publish_bestbranch_markers,
                                                       args=(self.node, filteredBestBranch, 1))
                    bestbranch_markers_thread.daemon = True
                    bestbranch_markers_thread.start()
                delaunayEdges = self.getDelaunayEdges(frontcones)
                if delaunayEdges:
                    if self.rviz_visualization:
                        delaunay_edges_thread = Thread(target=publish_delaunay_markers,
                                                       args=(self.node, delaunayEdges))
                        delaunay_edges_thread.daemon = True
                        delaunay_edges_thread.start()
                    newWaypoints = self.getWaypointsFromEdges(filteredBestBranch, delaunayEdges)
        if newWaypoints is not None:
            return newWaypoints
        else:
            return None

    def get_frontcones_with_color(self, frontdist):
        if len(self.cones) == 0:
            return []
        headingVector = self.getHeadingVector()  # 车辆前进方向

        carPosBehindPoint = np.array([self.egopose[0] - self.cfg.behindDist * headingVector[0],
                                      self.egopose[1] - self.cfg.behindDist * headingVector[1]])  # 车辆后方点

        frontcones = []
        frontconescolor = []
        for cone in self.cones:
            color = cone.color
            cone = np.array([cone.x, cone.y])
            vec = cone - carPosBehindPoint
            if np.dot(vec, headingVector) > 0:  # 桩筒在车辆前方
                dist = np.linalg.norm(cone - self.egopose[:2])
                if dist < frontdist:
                    angle = math.acos(np.dot(headingVector, vec) /
                                      (np.linalg.norm(headingVector) * np.linalg.norm(vec)))
                    sign = np.cross(headingVector, vec)
                    angle = angle * 180 / math.pi * sign
                    if color == ConeColor.ORANGE_BIG.value:
                        pass
                    elif dist > frontdist * self.cfg.nearRatio or angle < self.cfg.maxAngleDiff:
                        color = ConeColor.UNKNOWN.value
                    else:
                        if np.cross(headingVector, vec) > 0:
                            color = ConeColor.BLUE.value
                        else:
                            color = ConeColor.YELLOW.value
                    frontcones.append(cone)
                    frontconescolor.append(color)

        return frontcones, frontconescolor

    def getHeadingVector(self):
        headingVector = np.array([1.0, 0])
        yaw = self.egopose[2]
        carRotMat = np.array([[math.cos(yaw), -math.sin(yaw)],
                              [math.sin(yaw), math.cos(yaw)]])
        headingVector = carRotMat @ headingVector
        return headingVector

    @staticmethod
    def getDelaunayEdges(frontCones):
        if len(frontCones) < 4:  # no sense to calculate delaunay
            return
        conePoints = np.array(frontCones)

        tri = Delaunay(conePoints)
        delaunayEdges = []
        for simp in tri.simplices:
            for i in range(3):
                j = i + 1
                if j == 3:
                    j = 0
                edge = Edge(conePoints[simp[i]][0], conePoints[simp[i]][1],
                            conePoints[simp[j]][0], conePoints[simp[j]][1])

                if edge not in delaunayEdges:
                    delaunayEdges.append(edge)

        return delaunayEdges

    def getWaypointsFromEdges(self, filteredBranch, delaunayEdges):
        if not delaunayEdges:
            return

        left_count = np.zeros(len(delaunayEdges))  # 对每条delaunayEdge，统计其两个端点在左侧的数量
        right_count = np.zeros(len(delaunayEdges))  # 对每条delaunayEdge，统计其两个端点在右侧的数量
        for i in range(len(filteredBranch) - 1):
            node1 = filteredBranch[i]
            node2 = filteredBranch[i + 1]
            # 利用filteredBranch大致判断桩筒在左侧还是右侧
            for j in range(len(delaunayEdges)):
                edge = delaunayEdges[j]
                vertex1 = np.array([edge.x1, edge.y1])
                if self.isLeft(node2, node1, vertex1):
                    left_count[j] += 1
                else:
                    right_count[j] += 1
                vertex2 = np.array([edge.x2, edge.y2])
                if self.isLeft(node2, node1, vertex2):
                    left_count[j] += 1
                else:
                    right_count[j] += 1
        maxdiff = max(abs(left_count - right_count))
        mindiff = min(abs(left_count - right_count))
        threhold = int(self.cfg.maxSideDiffFactor * maxdiff + (1 - self.cfg.maxSideDiffFactor) * mindiff)
        wpCandidates = []
        for i in range(len(filteredBranch) - 1):
            node1 = filteredBranch[i]
            node2 = filteredBranch[i + 1]
            a1 = np.array([node1.x, node1.y])
            a2 = np.array([node2.x, node2.y])
            intersectedEdges = []
            for j in range(len(delaunayEdges)):
                edge = delaunayEdges[j]
                if abs(left_count[j] - right_count[j]) < threhold:
                    b1 = np.array([edge.x1, edge.y1])
                    b2 = np.array([edge.x2, edge.y2])

                    if self.getLineSegmentIntersection(a1, a2, b1, b2):
                        if edge.length() < self.cfg.maxAcceptedEdgeLength:
                            intersection = self.getLineIntersection(a1, a2, b1, b2)
                            edgePartsRatio = edge.getPartsLengthRatio(intersection)

                            if edgePartsRatio < self.cfg.maxEdgePartsRatio:
                                intersectedEdges.append(edge)
            if intersectedEdges:
                if len(intersectedEdges) > 1:
                    intersectedEdges.sort(key=lambda e: self.dist(node1.x, node1.y,
                                                                  e.intersection[0], e.intersection[1]))
                for edge in intersectedEdges:
                    wpCandidates.append(edge.getMiddlePoint())
        if wpCandidates:
            if len(wpCandidates) < 3:
                return wpCandidates
            else:
                waypoints = [wpCandidates[0], wpCandidates[1]]
                for i in range(2, len(wpCandidates)):
                    point1 = wpCandidates[i - 2]
                    point2 = wpCandidates[i - 1]
                    point3 = wpCandidates[i]
                    vec1 = point2 - point1
                    norm1 = np.linalg.norm(vec1)
                    vec2 = point3 - point2
                    norm2 = np.linalg.norm(vec2)
                    if norm1 < 0.001 or norm2 < 0.001:
                        waypoints.append(point3)
                        continue
                    angle = math.acos(np.dot(vec1, vec2) / (norm1 * norm2))
                    angle = angle * 180 / math.pi
                    if abs(angle) < self.cfg.maxwpAngleDiff:
                        waypoints.append(point3)
                return waypoints
        else:
            return None

    @staticmethod
    def getLineIntersection(a1, a2, b1, b2):
        """
        Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
        a1: [x, y] a point on the first line
        a2: [x, y] another point on the first line
        b1: [x, y] a point on the second line
        b2: [x, y] another point on the second line
        https://stackoverflow.com/questions/3252194/numpy-and-line-intersections
        """
        s = np.vstack([a1, a2, b1, b2])  # s for stacked
        h = np.hstack((s, np.ones((4, 1))))  # h for homogeneous
        l1 = np.cross(h[0], h[1])  # get first line
        l2 = np.cross(h[2], h[3])  # get second line
        x, y, z = np.cross(l1, l2)  # point of intersection
        if z == 0:  # lines are parallel
            return float('inf'), float('inf')
        return x / z, y / z

    def getLineSegmentIntersection(self, a1, a2, b1, b2):
        # https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
        # Return true if line segments a1a2 and b1b2 intersect
        return self.ccw(a1, b1, b2) != self.ccw(a2, b1, b2) and self.ccw(a1, a2, b1) != self.ccw(a1, a2, b2)

    @staticmethod
    def ccw(A, B, C):
        # if three points are listed in a counterclockwise order.
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    @staticmethod
    def dist(x1, y1, x2, y2):
        distSq = (x1 - x2) ** 2 + (y1 - y2) ** 2
        return math.sqrt(distSq)

    def findBestBranch(self, leafNodes, nodeList, largerGroupfrontcones, largerGroupfrontconescolor, direction):
        if not leafNodes:
            return

        leafRatings = []  # 保存每条路径的评分
        for leaf in leafNodes:  # 遍历每条到达规划距离的路径
            heading_vec = self.getHeadingVector()
            last_node_pos = np.array([leaf.x, leaf.y])
            last_node_vec = last_node_pos - self.egopose[:2]
            angle = np.arcsin(np.cross(heading_vec, last_node_vec) /
                              np.linalg.norm(heading_vec) / np.linalg.norm(last_node_vec))
            if direction == 'right':
                if angle > -0.5:
                    leafRatings.append(-100.0)
                    continue
            elif direction == 'left':
                if angle < 0.5:
                    leafRatings.append(-100.0)
                    continue
            elif direction == 'forward':
                if abs(angle) > 0.15:
                    leafRatings.append(-100.0)
                    continue
            else:
                raise ValueError('direction should be right or left')

            branchRating = 0  # 路径的评分
            node = leaf  # 从路径的终点往前遍历
            while node.parent is not None:
                nodeRating = 0  # 单节点的评分

                leftCones = []
                rightCones = []
                for i in range(len(largerGroupfrontcones)):
                    cone = largerGroupfrontcones[i]
                    node_pos = np.array([node.x, node.y])
                    coneDist = np.linalg.norm(cone - node_pos)
                    if coneDist < self.cfg.coneDistLimit:
                        if coneDist < self.cfg.coneObstacleSize:
                            continue
                        nodeRating += (self.cfg.coneDistLimit - coneDist)  # 离桩筒距离越近评分越高，目的是使路径点在赛道内部

                        cost_yaw = node.yaw * self.cfg.yawChangepenaltyFactor
                        nodeRating += cost_yaw  # 路径点的角度变化越小越好

                        if largerGroupfrontconescolor[i] == ConeColor.BLUE.value:
                            colorRating = 1.0
                        elif largerGroupfrontconescolor[i] == ConeColor.YELLOW.value:
                            colorRating = -1.0
                        else:
                            colorRating = 0.0
                        if self.isLeft(node, nodeList[node.parent], cone):  # 判断桩筒在路径的左侧还是右侧
                            leftCones.append(i)
                            nodeRating += colorRating * self.cfg.correctColorReward  # 若颜色正确，评分增加，否则减少，不确定则不变
                        else:
                            rightCones.append(i)
                            nodeRating -= colorRating * self.cfg.correctColorReward

                if (len(leftCones) == 0 and len(rightCones)) > 0 or (len(leftCones) > 0 and len(rightCones) == 0):
                    nodeRating = 0  # 若在赛道外部，评分为0

                nodeFactor = ((node.cost - self.cfg.expandDistance) /
                              (self.cfg.planDistance - self.cfg.expandDistance) + 1)  # 权重因子，越靠后的路径点权重越大
                branchRating += nodeRating * nodeFactor
                node = nodeList[node.parent]
            leafRatings.append(branchRating)

        if leafRatings:
            maxRating = max(leafRatings)
            if maxRating <= -100.0:
                return None
            maxRatingInd = leafRatings.index(maxRating)

            node = leafNodes[maxRatingInd]

            reverseBranch = [node]
            while node.parent is not None:
                node = nodeList[node.parent]
                reverseBranch.append(node)

            directBranch = []
            for n in reversed(reverseBranch):
                directBranch.append(n)

            return directBranch
        else:
            return None

    @staticmethod
    def isLeft(node, parentNode, cone):
        vec_path = np.array([node.x - parentNode.x, node.y - parentNode.y])
        vec_node_to_cone = np.array([cone[0] - node.x, cone[1] - node.y])
        is_left = np.cross(vec_path, vec_node_to_cone) > 0
        return is_left

    def getFilteredBestBranch(self, bestBranch):
        if not bestBranch:
            return

        if not self.filteredBestBranch:
            self.filteredBestBranch = list(bestBranch)
        else:
            shouldDiscard = False
            count = 0
            min_length = min(len(bestBranch), len(self.filteredBestBranch))
            bestBranch = bestBranch[:min_length]
            self.filteredBestBranch = self.filteredBestBranch[:min_length]
            for i in range(min_length):
                node = bestBranch[i]
                filteredNode = self.filteredBestBranch[i]

                dist = math.sqrt((node.x - filteredNode.x) ** 2 + (node.y - filteredNode.y) ** 2)
                if dist > self.cfg.everyPointDistChangeLimit:  # changed too much, skip this branch
                    count += 1
                if count >= self.cfg.resetCount:
                    shouldDiscard = True
                    break

            if shouldDiscard:
                self.filteredBestBranch = list(bestBranch)
            else:
                for i in range(min_length):  # soft update
                    self.filteredBestBranch[i].x = (self.filteredBestBranch[i].x * (1 - self.cfg.newPointFilter) +
                                                    self.cfg.newPointFilter * bestBranch[i].x)
                    self.filteredBestBranch[i].y = (self.filteredBestBranch[i].y * (1 - self.cfg.newPointFilter) +
                                                    self.cfg.newPointFilter * bestBranch[i].y)

        return list(self.filteredBestBranch)  # return copy

    def fit_circle(self, waypoints, direction='right'):
        if not waypoints:
            return
        newSavedPoints = []
        for i in range(len(waypoints)):
            waypointCandidate = waypoints[i]
            carWaypointDist = self.dist(self.egopose[0], self.egopose[1], waypointCandidate[0], waypointCandidate[1])
            if i >= self.cfg.maxWaypointAmountToSave or carWaypointDist > self.cfg.maxDistToSaveWaypoints:
                break
            else:
                newSavedPoints.append(waypointCandidate)
                self.saved_circle_points.append(waypointCandidate)
        if self.rviz_visualization:
            savedwaypoints_markers_thread = Thread(target=publish_savedwaypoints_markers,
                                                   args=(self.node, self.saved_circle_points))
            savedwaypoints_markers_thread.daemon = True
            savedwaypoints_markers_thread.start()
        if len(self.saved_circle_points) >= self.cfg.saved_circle_points_num_before_fitting:
            initial_guess = np.append(self.guessed_right_circle_center, 9.125)
            self.circle = ransac_circle(np.array(self.saved_circle_points), initial_guess, self.cfg)
            if self.rviz_visualization:
                circle_thread = Thread(target=publish_circle_markers, args=(self.node, self.circle))
                circle_thread.daemon = True
                circle_thread.start()
            waypoints = self.calculate_circle_waypoints(direction)
            return waypoints
        else:
            if newSavedPoints:  # make self.savedWaypoints and newWaypoints having no intersection
                for point in newSavedPoints:
                    waypoints.remove(point)
            return list(waypoints)

    def calculate_circle_waypoints(self, direction):
        if direction == 'right':
            angles = np.linspace(0, -2 * np.pi, self.cfg.total_fitted_circle_points, endpoint=False)
        elif direction == 'left':
            angles = np.linspace(0, 2 * np.pi, self.cfg.total_fitted_circle_points, endpoint=False)
        else:
            raise ValueError('direction should be right or left')
        points = np.zeros((self.cfg.total_fitted_circle_points, 2))

        if not self.last_radius:
            self.last_radius = self.circle[2]
        target_radius = self.last_radius + self.cfg.radius_change_factor * (self.cfg.target_radius - self.last_radius)
        points[:, 0] = self.circle[0] + target_radius * np.cos(angles)
        points[:, 1] = self.circle[1] + target_radius * np.sin(angles)
        self.last_radius = target_radius

        if not self.idx:
            w_size = len(points)
            self.idx = self.closest_wp_idx(points, 0, w_size)
        else:
            start_idx = (self.idx - 10) % len(points)
            self.idx = self.closest_wp_idx(points, start_idx)
        indices = []
        for i in range(self.cfg.pub_wp_num):
            indices.append((self.idx + i) % len(points))
        WaypointstoPublish = [points[i] for i in indices]
        return WaypointstoPublish

    def closest_wp_idx(self, points, f_idx, w_size=20):
        min_dist = float("inf")  # 保存最近的距离
        closest_wp_index = 0  # default WP
        for i in range(w_size):
            if f_idx + i >= len(points):
                i = i - len(points)
            temp_wp = points[f_idx + i]
            heading_vec = self.getHeadingVector()
            vec = temp_wp - self.egopose[:2]
            temp_dist = np.linalg.norm(vec)
            if temp_dist <= min_dist \
                    and np.dot(vec, heading_vec) > 0:
                closest_wp_index = i
                min_dist = temp_dist
        return f_idx + closest_wp_index


class Edge:
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.intersection = None

    def getMiddlePoint(self):
        return np.array([(self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2])

    def length(self):
        return math.sqrt((self.x1 - self.x2) ** 2 + (self.y1 - self.y2) ** 2)

    def getPartsLengthRatio(self, intersection):
        self.intersection = intersection
        part1Length = math.sqrt((self.x1 - intersection[0]) ** 2 + (self.y1 - intersection[1]) ** 2)
        part2Length = math.sqrt((intersection[0] - self.x2) ** 2 + (intersection[1] - self.y2) ** 2)

        return max(part1Length, part2Length) / min(part1Length, part2Length)
