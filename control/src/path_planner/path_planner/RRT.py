import copy
import math
import random
import numpy as np


class RRT:
    """
    Class for RRT Planning
    """

    def __init__(self, start, cfg, obstacleList, rrtTargets=None):
        self.start = Node(start[0], start[1], start[2])
        self.startYaw = start[2]

        self.cfg = cfg

        self.obstacleList = obstacleList
        self.rrtTargets = rrtTargets

    def Planning(self):
        self.nodeList = [self.start]
        self.leafNodes = []  # 用于储存到达目标距离的节点

        for i in range(self.cfg.iterationNumber):
            rnd = self.get_random_point_from_target_list()  # 从target附近随机选取一个点
            nind = self.GetNearestListIndex(rnd)  # 从nodeList中找到离rnd最近的节点的索引
            nearestNode = self.nodeList[nind]

            if nearestNode.cost >= self.cfg.planDistance:  # 已经到达目标距离
                continue

            newNode = self.steerConstrained(rnd, nind)

            # due to angle constraints it is possible that similar node is generated
            if newNode in self.nodeList:
                continue

            if self.__CollisionCheck(newNode):
                nearinds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearinds)
                self.nodeList.append(newNode)
                self.rewire(newNode, nearinds)

                if newNode.cost >= self.cfg.planDistance:
                    self.leafNodes.append(newNode)

        return self.nodeList, self.leafNodes

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            node = self.nodeList[i]
            dx = newNode.x - node.x
            dy = newNode.y - node.y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(node, theta, d):
                dlist.append(node.cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            return newNode

        newNode.cost = mincost
        newNode.parent = minind

        return newNode

    def steerConstrained(self, rnd, nind):
        # expand tree
        nearestNode = self.nodeList[nind]
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

        # 角度变化限制
        angleChange = self.pi_2_pi(theta - nearestNode.yaw)

        anglelimit = math.radians(self.cfg.expandAngle)

        if angleChange > anglelimit:
            angleChange = anglelimit
        elif angleChange >= -anglelimit:
            angleChange = 0
        else:
            angleChange = -anglelimit

        newNode = copy.deepcopy(nearestNode)
        newNode.yaw += angleChange
        newNode.x += self.cfg.expandDistance * math.cos(newNode.yaw)
        newNode.y += self.cfg.expandDistance * math.sin(newNode.yaw)

        newNode.cost += self.cfg.expandDistance
        newNode.parent = nind

        return newNode

    @staticmethod
    def pi_2_pi(angle):
        """
        用于将角度转换到-pi到pi之间
        """
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def get_random_point(self):
        randX = random.uniform(0, self.cfg.planDistance)
        randY = random.uniform(-self.cfg.planDistance, self.cfg.planDistance)
        rnd = [randX, randY]

        car_rot_mat = np.array([[math.cos(self.startYaw), -math.sin(self.startYaw)],
                                [math.sin(self.startYaw), math.cos(self.startYaw)]])
        rotatedRnd = np.dot(car_rot_mat, rnd)

        rotatedRnd = [rotatedRnd[0] + self.start.x, rotatedRnd[1] + self.start.y]

        return rotatedRnd

    def get_random_point_from_target_list(self):
        if not self.rrtTargets:
            return self.get_random_point()

        targetId = np.random.randint(len(self.rrtTargets))
        x, y, oSize = self.rrtTargets[targetId]

        # square idea
        # randX = random.uniform(-maxTargetAroundDist, maxTargetAroundDist)
        # randY = random.uniform(-maxTargetAroundDist, maxTargetAroundDist)
        # finalRnd = [x + randX, y + randY]

        # circle idea
        randAngle = random.uniform(0, 2 * math.pi)
        randDist = random.uniform(oSize, self.cfg.maxTargetAroundDist)
        finalRnd = [x + randDist * math.cos(randAngle), y + randDist * math.sin(randAngle)]

        return finalRnd

    def find_near_nodes(self, newNode):
        r = self.cfg.expandDistance * self.cfg.nearSearchRatio
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]

        return nearinds

    def rewire(self, newNode, nearinds):
        nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]

            dx = nearNode.x - newNode.x
            dy = nearNode.y - newNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = newNode.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nearNode, theta, d):
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost
                    self.propagate_cost_to_leaves(i)

    def propagate_cost_to_leaves(self, parent_node_ind):
        for i in range(len(self.nodeList)):
            node = self.nodeList[i]
            if node.parent == parent_node_ind:
                parent_node = self.nodeList[parent_node_ind]
                dx = parent_node.x - node.x
                dy = parent_node.y - node.y
                d = math.sqrt(dx ** 2 + dy ** 2)
                node.cost = parent_node.cost + d
                self.propagate_cost_to_leaves(i)

    def check_collision_extend(self, nearNode, theta, d):

        tmpNode = copy.deepcopy(nearNode)

        for i in range(int(d / self.cfg.expandDistance)):
            tmpNode.x += self.cfg.expandDistance * math.cos(theta)
            tmpNode.y += self.cfg.expandDistance * math.sin(theta)
            if not self.__CollisionCheck(tmpNode):
                return False

        return True

    def GetNearestListIndex(self, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in self.nodeList]
        minind = dlist.index(min(dlist))
        return minind

    def __CollisionCheck(self, node):
        for (ox, oy, size) in self.obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = dx * dx + dy * dy
            if d <= size ** 2:
                return False  # collision
        return True  # safe


class Node:
    """
    RRT Node
    """
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.cost = 0.0
        self.parent = None
