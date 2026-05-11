import numpy as np
import math
from threading import Thread
from std_msgs.msg import Bool
from scipy.spatial import Delaunay
from .RRT import RRT
from .visualization import (publish_tree_markers, publish_bestbranch_markers,
                            publish_delaunay_markers, publish_savedwaypoints_markers)


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
        # 评分函数
        self.coneDistLimit = 4.0  # 桩筒距离限制(一般可设置为赛道宽度)
        self.correctColorReward = 1.0  # 颜色正确奖励
        self.yawChangepenaltyFactor = 1.0  # 角度变化惩罚
        # 平滑路径
        self.everyPointDistChangeLimit = 2.0  # 每个路径点的距离变化限制
        self.newPointFilter = 0.2  # 新路径点的滤波因子
        self.resetCount = 2  # 重置计数
        # Delaunay
        self.maxAcceptedEdgeLength = 7.0  # 最大接受边长
        self.maxEdgePartsRatio = 3.0  # 交点分割边长最大比例
        self.maxSideDiffFactor = 0.8  # 左右桩筒数量差异系数
        self.maxwpAngleDiff = 50.0  # 路径点角度差异
        # 合并路径
        self.maxDistToSaveWaypoints = 2.0  # 最大保存路径点距离
        self.maxWaypointAmountToSave = 2  # 最大保存路径点数量
        self.waypointsDistTolerance = 0.5  # 路径点距离容差
        self.minDisttoPublish = 3.0  # 最小发布距离
        self.minWpNum = 3  # 最小发布路径点数量
        # loopClosure
        self.loopPreliminaryCloseureTolerance = 1.0  # 初步闭环容差
        self.loopCloseureTolerance = 3.0  # 闭环容差
        


class MidLinePlanner:
    def __init__(self, node):
        self.rviz_visualization = node.rviz_visualization
        self.cfg = PlannerCFG()
        self.node = node
        self.cones = []  # 桩筒坐标
        self.egopose = np.array([0.0, 0.0, 0.0])  # 车辆位置(x, y, yaw)
        self.filteredBestBranch = []
        self.savedWaypoints = []  # 已保存的所有路径
        self.preliminaryLoopClosure = False
        self.loopClosure = False  # 路径是否已闭环
        self.idx = 0
        # 新增状态变量
        self.recovery_mode = False
        self.consecutive_deviations = 0  # 连续偏离计数
        self.last_valid_path = None  # 最后有效路径
        self.max_lateral_deviation = 2.5  # 最大允许横向偏差（米）

    def update_map(self, cones):
        if len(cones) >= 2:
            self.cones = cones

    def update_egopose(self, egopose):
        self.egopose = egopose

    def plan(self):
        if len(self.cones) == 0:
            return None
        frontCones, is_left_cone = self.getFrontConeObstacles(self.cfg.frontConesDist)
        
        # 条件1：前方锥桶左右分布异常检测
        left_cones = sum(1 for x in is_left_cone if x == 1)
        right_cones = sum(1 for x in is_left_cone if x == -1)
        cone_imbalance = abs(left_cones - right_cones) / (len(frontCones)+1e-5) > 0.8
        
        # 条件2：横向路径偏差检测
        lateral_deviation = self.calculate_lateral_deviation()
        path_deviation = lateral_deviation > self.max_lateral_deviation
        
        # 综合偏离判断
        if (cone_imbalance or path_deviation) and len(frontCones)>=2:
            self.consecutive_deviations += 1
            if self.consecutive_deviations > 3:  # 连续3次检测触发
                self.recovery_mode = True
                self.last_valid_path = self.savedWaypoints[-10:]  # 保存最近有效路径
        else:
            self.consecutive_deviations = max(0, self.consecutive_deviations-1)
            self.recovery_mode = False
        print("plan",self.recovery_mode)
        # 恢复模式路径生成
        if self.recovery_mode:
            return self.generate_guided_recovery_path()
        if self.savedWaypoints:
            self.idx = self.closest_wp_idx(self.idx)  # 更新自车在路径上的位置
        if len(self.cones) > 0:
            waypoints = self.sampleTree()
            return waypoints
        else:
            return None
    
    def calculate_lateral_deviation(self):
        """计算与规划路径的横向偏差"""
        if not self.savedWaypoints:
            return 0
            
        closest_idx = self.closest_wp_idx(self.idx)
        path_vector = self.savedWaypoints[closest_idx] - self.egopose[:2]
        heading_vector = self.getHeadingVector()
        cross_product = np.cross(heading_vector, path_vector)
        return abs(cross_product / np.linalg.norm(heading_vector))

    def generate_guided_recovery_path(self):
        """生成向前行驶的渐进回归路径"""
        if not self.last_valid_path:
            return None
            
        RECOVERY_STEPS = 8  # 增加路径点数
        recovery_path = []
        current_pose = self.egopose.copy()
        
        # 计算目标方向（历史路径的平均方向与当前航向的加权）
        valid_vectors = np.diff(self.last_valid_path, axis=0)
        avg_direction = np.mean(valid_vectors, axis=0) if len(valid_vectors)>0 else self.getHeadingVector()
        target_yaw = np.arctan2(avg_direction[1], avg_direction[0])
        
        # 混合权重：70%历史方向，30%当前航向
        target_yaw = 0.7*target_yaw + 0.3*current_pose[2]
        
        for step in range(RECOVERY_STEPS):
            # 动态调整转向（限制最大转向率）
            yaw_error = self._wrap_angle(target_yaw - current_pose[2])
            steering = np.clip(yaw_error * 0.5, -0.35, 0.35)
            
            # 生成路径点（强制保持前进）
            current_pose[2] += steering
            current_pose[0] += 3.0 * np.cos(current_pose[2])  # 固定前进距离
            current_pose[1] += 3.0 * np.sin(current_pose[2])
            recovery_path.append(np.array(current_pose[:2]))
            
            # 提前退出条件
            if self._check_return_track(current_pose):
                break
                
        return recovery_path

    def _wrap_angle(self, angle):
        """角度归一化到[-π, π]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def _check_return_track(self, pose):
        """检查是否回到赛道"""
        if not self.savedWaypoints:
            return False
        closest_idx = self.closest_wp_idx(0)
        return np.linalg.norm(pose[:2] - self.savedWaypoints[closest_idx]) < 1.5
    
    def closest_wp_idx(self, f_idx, w_size=10):
        # 改进的路径点选择（考虑航向一致性）
        best_idx = f_idx
        max_forward_weight = 0
        heading_vec = self.getHeadingVector()
        
        for i in range(w_size):
            idx = (f_idx + i) % len(self.savedWaypoints)
            wp = self.savedWaypoints[idx]
            vec = wp - self.egopose[:2]
            
            forward_component = np.dot(vec, heading_vec)
            lateral_component = np.linalg.norm(np.cross(vec, heading_vec))
            
            # 综合评分公式
            score = forward_component - 0.5 * lateral_component
            
            if score > max_forward_weight:
                max_forward_weight = score
                best_idx = idx
                
        return best_idx
    
    # def closest_wp_idx(self, f_idx, w_size=10):
    #     min_dist = float("inf")  # 保存最近的距离
    #     closest_wp_index = 0  # default WP
    #     if not self.loopClosure:
    #         w_size = w_size if w_size <= len(self.savedWaypoints) - f_idx else len(self.savedWaypoints) - f_idx
    #     for i in range(w_size):
    #         if f_idx + i >= len(self.savedWaypoints):
    #             i = i - len(self.savedWaypoints)
    #         temp_wp = self.savedWaypoints[f_idx + i]
    #         heading_vec = self.getHeadingVector()
    #         vec = temp_wp - self.egopose[:2]
    #         temp_dist = np.linalg.norm(vec)
    #         if temp_dist <= min_dist \
    #                 and np.dot(vec, heading_vec) > 0:
    #             closest_wp_index = i
    #             min_dist = temp_dist
    #     return f_idx + closest_wp_index

    def sampleTree(self):
        if len(self.cones) == 0:
            return None

        frontCones, _ = self.getFrontConeObstacles(self.cfg.frontConesDist)

        coneObstacleList = []
        rrtConeTargets = []  # rrt目标
        for cone in frontCones:
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
            
        largerGroupFrontCones, is_left_cone_larger = self.getFrontConeObstacles(self.cfg.frontConesBiggerDist)
        # 寻找最佳路径
        bestBranch = self.findBestBranch(leafNodes, nodeList, largerGroupFrontCones, is_left_cone_larger)
        if bestBranch:
            if self.rviz_visualization:
                bestbranch_markers_thread = Thread(target=publish_bestbranch_markers, args=(self.node, bestBranch, 0))
                bestbranch_markers_thread.daemon = True
                bestbranch_markers_thread.start()
            filteredBestBranch = self.getFilteredBestBranch(bestBranch)
            if filteredBestBranch:
                if self.rviz_visualization:
                    bestbranch_markers_thread = Thread(target=publish_bestbranch_markers,
                                                       args=(self.node, filteredBestBranch, 1))
                    bestbranch_markers_thread.daemon = True
                    bestbranch_markers_thread.start()
                # Delaunay
                delaunayEdges = self.getDelaunayEdges(frontCones)

                newWaypoints = []
                if delaunayEdges:
                    if self.rviz_visualization:
                        delaunay_edges_thread = Thread(target=publish_delaunay_markers, args=(self.node, delaunayEdges))
                        delaunay_edges_thread.daemon = True
                        delaunay_edges_thread.start()
                    newWaypoints = self.getWaypointsFromEdges(filteredBestBranch, delaunayEdges)
                if newWaypoints:
                    WaypointstoPublish = self.mergeWaypoints(newWaypoints)
                    if WaypointstoPublish:
                        total_distance = 0
                        for i in range(len(WaypointstoPublish) - 1):
                            total_distance += np.linalg.norm(WaypointstoPublish[i] - WaypointstoPublish[i + 1])
                        if total_distance > self.cfg.minDisttoPublish and len(WaypointstoPublish) > self.cfg.minWpNum:
                            return WaypointstoPublish
                    return None
        elif self.loopClosure:
            WaypointstoPublish = self.mergeWaypoints(None)
            return WaypointstoPublish
        return None

    def getFrontConeObstacles(self, frontDist):
        if len(self.cones) == 0:
            return []

        headingVector = self.getHeadingVector()  # 车辆前进方向

        carPosBehindPoint = np.array([self.egopose[0] - self.cfg.behindDist * headingVector[0],
                                      self.egopose[1] - self.cfg.behindDist * headingVector[1]])  # 车辆后方点

        frontConeList = []
        is_left_cone = []

        # 使用感知的相机颜色信息
        # for cone in self.cones:
        #     color = cone.color
        #     if color == cone.BLUE:
        #         is_left = 1
        #     elif color == cone.YELLOW:
        #         is_left = -1
        #     else:
        #         is_left = 0
        #     cone = np.array([cone.x, cone.y])
        #     vec = cone - carPosBehindPoint
        #     if np.dot(vec, headingVector) > 0:  # 桩筒在车辆前方
        #         dist = np.linalg.norm(cone - self.egopose[:2])
        #         if dist < frontDist:
        #             frontConeList.append(cone)
        #             is_left_cone.append(is_left)

        # 使用桩筒相对自车的位置
        for cone in self.cones:
            cone = np.array([cone.x, cone.y])
            vec = cone - carPosBehindPoint
            if np.dot(vec, headingVector) > 0:
                dist = np.linalg.norm(cone - self.egopose[:2])
                if dist < frontDist:
                    frontConeList.append(cone)
                    # 计算headingVector与vec的夹角和方向
                    angle = math.acos(np.dot(headingVector, vec) /
                                      (np.linalg.norm(headingVector) * np.linalg.norm(vec)))
                    sign = np.cross(headingVector, vec)
                    angle = angle * 180 / math.pi * sign
                    if dist > frontDist * self.cfg.nearRatio or angle < self.cfg.maxAngleDiff:
                        # 若夹角较小或距离过长，则不判断桩筒的左右位置
                        is_left_cone.append(0)
                    else:
                        if np.cross(headingVector, vec) > 0:
                            is_left_cone.append(1)
                        else:
                            is_left_cone.append(-1)

        return frontConeList, is_left_cone

    def getHeadingVector(self):
        headingVector = np.array([1.0, 0])
        yaw = self.egopose[2]
        carRotMat = np.array([[math.cos(yaw), -math.sin(yaw)],
                              [math.sin(yaw), math.cos(yaw)]])
        headingVector = carRotMat @ headingVector
        return headingVector

    def findBestBranch(self, leafNodes, nodeList, largerGroupFrontCones, is_left_cone):
        if not leafNodes:
            return

        leafRatings = []  # 保存每条路径的评分
        for leaf in leafNodes:  # 遍历每条到达规划距离的路径
            branchRating = 0  # 路径的评分
            node = leaf  # 从路径的终点往前遍历
            while node.parent is not None:
                nodeRating = 0  # 单节点的评分

                leftCones = []
                rightCones = []
                for i in range(len(largerGroupFrontCones)):
                    cone = largerGroupFrontCones[i]
                    node_pos = np.array([node.x, node.y])
                    coneDist = np.linalg.norm(cone - node_pos)
                    if coneDist < self.cfg.coneDistLimit:
                        if coneDist < self.cfg.coneObstacleSize:
                            continue
                        nodeRating += (self.cfg.coneDistLimit - coneDist)  # 离桩筒距离越近评分越高，目的是使路径点在赛道内部

                        cost_yaw = -abs(node.yaw) * self.cfg.yawChangepenaltyFactor
                        nodeRating += cost_yaw  # 路径点的角度变化越小越好

                        if self.isLeft(node, nodeList[node.parent], cone):  # 判断桩筒在路径的左侧还是右侧
                            leftCones.append(i)
                            nodeRating += is_left_cone[i] * self.cfg.correctColorReward  # 若颜色正确，评分增加，否则减少，不确定则不变
                        else:
                            rightCones.append(i)
                            nodeRating -= is_left_cone[i] * self.cfg.correctColorReward

                if (len(leftCones) == 0 and len(rightCones)) > 0 or (len(leftCones) > 0 and len(rightCones) == 0):
                    nodeRating = 0  # 若在赛道外部，评分为0

                nodeFactor = ((node.cost - self.cfg.expandDistance) /
                              (self.cfg.planDistance - self.cfg.expandDistance) + 1)  # 权重因子，越靠后的路径点权重越大
                branchRating += nodeRating * nodeFactor
                node = nodeList[node.parent]
            leafRatings.append(branchRating)

        maxRating = max(leafRatings)
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

    def mergeWaypoints(self, newWaypoints):
        if not self.loopClosure:
            if not newWaypoints:
                return
            # check preliminary loopClosure
            if len(self.savedWaypoints) > 15:
                firstSavedWaypoint = self.savedWaypoints[0]
                for waypoint in reversed(newWaypoints):
                    distDiff = self.dist(firstSavedWaypoint[0], firstSavedWaypoint[1], waypoint[0], waypoint[1])
                    if distDiff < self.cfg.loopCloseureTolerance:
                        self.preliminaryLoopClosure = True
                        print("Preliminary Loop Closure!!")
                        break

            newSavedPoints = []
            idx = self.idx
            for i in range(len(newWaypoints)):
                waypointCandidate = newWaypoints[i]
                carWaypointDist = self.dist(self.egopose[0], self.egopose[1],
                                            waypointCandidate[0], waypointCandidate[1])
                if i >= self.cfg.maxWaypointAmountToSave or carWaypointDist > self.cfg.maxDistToSaveWaypoints:
                    break
                else:
                    if self.preliminaryLoopClosure:
                        distDiff = self.dist(firstSavedWaypoint[0], firstSavedWaypoint[1],
                                             waypointCandidate[0], waypointCandidate[1])
                        if distDiff < self.cfg.loopCloseureTolerance:
                            self.loopClosure = True
                            print("Loop closed!!")
                            msg = Bool()
                            msg.data = True
                            self.node.state_indicator_pub.publish(msg)
                            break

                    start_idx = idx - 2 if idx >= 2 else 0
                    min_dist = float("inf")
                    closest_wp_index = start_idx
                    dist = 0
                    for j in range(start_idx, start_idx + 5):
                        if j >= len(self.savedWaypoints):
                            break
                        savedWaypoint = self.savedWaypoints[j]
                        vec = waypointCandidate - savedWaypoint
                        heading_vec = self.getHeadingVector()
                        sign = 1 if np.dot(vec, heading_vec) > 0 else -1
                        dist = np.linalg.norm(vec) * sign
                        if abs(dist) <= min_dist:
                            closest_wp_index = j
                            min_dist = abs(dist)
                    idx = closest_wp_index
                    if min_dist < self.cfg.waypointsDistTolerance:
                        self.savedWaypoints[closest_wp_index] = (self.cfg.newPointFilter * waypointCandidate +
                                                                 (1 - self.cfg.newPointFilter) *
                                                                 self.savedWaypoints[closest_wp_index])
                        newSavedPoints.append(waypointCandidate)
                    else:
                        if dist >= 0:
                            if closest_wp_index >= len(self.savedWaypoints) - 1:
                                self.savedWaypoints.append(waypointCandidate)
                            else:
                                self.savedWaypoints.insert(closest_wp_index + 1, waypointCandidate)
                        else:
                            self.savedWaypoints.insert(closest_wp_index, waypointCandidate)
                        newSavedPoints.append(waypointCandidate)
            if self.rviz_visualization:
                savedwaypoints_markers_thread = Thread(target=publish_savedwaypoints_markers,
                                                       args=(self.node, self.savedWaypoints))
                savedwaypoints_markers_thread.daemon = True
                savedwaypoints_markers_thread.start()
            if newSavedPoints:  # make self.savedWaypoints and newWaypoints having no intersection
                for point in newSavedPoints:
                    newWaypoints.remove(point)
            return list(newWaypoints)
        else:
            if not newWaypoints:
                closest_wp_index = self.idx - 2
            else:
                idx = self.idx
                closest_wp_index = idx - 2
                for i in range(len(newWaypoints)):
                    waypointCandidate = newWaypoints[i]
                    carWaypointDist = self.dist(self.egopose[0], self.egopose[1],
                                                waypointCandidate[0], waypointCandidate[1])
                    if i >= self.cfg.maxWaypointAmountToSave or carWaypointDist > self.cfg.maxDistToSaveWaypoints:
                        break
                    else:
                        start_idx = idx - 2
                        min_dist = float("inf")
                        closest_wp_index = start_idx
                        for j in range(start_idx, start_idx + 5):
                            if j >= len(self.savedWaypoints):
                                j = j % len(self.savedWaypoints)
                            savedWaypoint = self.savedWaypoints[j]
                            pre_savedWaypoint = self.savedWaypoints[j - 1]
                            vec1 = waypointCandidate - savedWaypoint
                            vec2 = waypointCandidate - pre_savedWaypoint
                            dist = np.linalg.norm(vec1) + np.linalg.norm(vec2)
                            if dist <= min_dist:
                                closest_wp_index = j
                                min_dist = dist
                        idx = closest_wp_index
                        vec1 = waypointCandidate - self.savedWaypoints[closest_wp_index]
                        vec2 = waypointCandidate - self.savedWaypoints[closest_wp_index - 1]
                        heading_vec = self.savedWaypoints[closest_wp_index] - self.savedWaypoints[closest_wp_index - 1]
                        dist1 = np.linalg.norm(vec1)
                        dist2 = np.linalg.norm(vec2)
                        if min(dist1, dist2) < self.cfg.waypointsDistTolerance * 2.0:
                            if dist1 < dist2:
                                update_idx = closest_wp_index
                            else:
                                update_idx = closest_wp_index - 1
                            self.savedWaypoints[update_idx] = (self.cfg.newPointFilter * waypointCandidate +
                                                               (1 - self.cfg.newPointFilter) *
                                                               self.savedWaypoints[update_idx])
                        elif np.dot(vec1, heading_vec) < 0:
                            self.savedWaypoints.insert(closest_wp_index, waypointCandidate)
                        else:
                            if closest_wp_index >= len(self.savedWaypoints) - 1:
                                self.savedWaypoints.insert(0, waypointCandidate)
                            else:
                                self.savedWaypoints.insert(closest_wp_index + 1, waypointCandidate)
            if closest_wp_index == self.idx - 2:  # no valid new waypoints
                return None
            if self.rviz_visualization:
                savedwaypoints_markers_thread = Thread(target=publish_savedwaypoints_markers,
                                                       args=(self.node, self.savedWaypoints))
                savedwaypoints_markers_thread.daemon = True
                savedwaypoints_markers_thread.start()
            pub_idx_start = closest_wp_index
            pub_wp_num = 8
            indices = []
            for i in range(pub_wp_num):
                indices.append((pub_idx_start + i) % len(self.savedWaypoints))
            WaypointstoPublish = [self.savedWaypoints[i] for i in indices]
            return list(WaypointstoPublish)


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
