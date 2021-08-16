import copy
import math
import random
import time
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
import numpy as np
import calculate
import common
import openpyxl
import pandas as pd
from pandas import DataFrame


class RRT:

    def __init__(self, obstacleList, randArea, y=2, alpha=0.97, t=0,
                 expandDis=2.0, goalSampleRate=10, p1=1.6, p2=1.8, maxIter=None):

        self.start = None
        self.goal = None
        self.min_rand = randArea[0]  # 最小采样范围
        self.max_rand = randArea[1]  # 最大采样范围
        self.expand_dis = expandDis
        self.goal_sample_rate = goalSampleRate  # 每次采样有百分之多少的概率，采样到终点，这里设置10%；目标偏置思想
        self.max_iter = maxIter
        self.obstacle_list = obstacleList
        self.node_list = None
        self.alpha = alpha  # 步长因子
        self.t = t  # 迭代次数
        self.p1 = p1
        self.p2 = p2
        self.y = y

    #######################
    # 自适应步长函数； 参考论文的，暂无使用
    #######################
    def alphat(self, t):
        next_alpha = self.alpha * (1 - t / self.max_iter)
        return next_alpha

    #######################
    # 得到引力系数k； 参考论文的，暂无使用
    #######################
    @staticmethod
    def get_k(node1, node2):
        k = math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)
        return k

    #######################
    # 以起始点和目标点连线得到的中点为圆心(point)，半径(radius)的圆内均匀采样
    # random()产生一个0-1之间的的随机浮点数
    #######################
    @staticmethod
    def circle_sample(point, radius):
        random_r = radius * math.sqrt(random.random())  # 开方，保证均匀采样
        random_theta = 2 * math.pi * random.random()  # 弧度
        return [(point[0] + random_r * math.sin(random_theta)), (point[1] + random_r * math.cos(random_theta))]

    #######################
    # 用来显示障碍物的坐标；暂时不用
    # 经测试，OK
    #######################
    @staticmethod
    def show_obstacle(obstacleList):
        barrier_number = len(obstacleList)
        print("障碍物的个数", barrier_number)
        for i in range(barrier_number):
            show_x = obstacleList[i]
            print("x坐标", obstacleList[i][0])
            show_y = obstacleList[i]
            print("y坐标", obstacleList[i][1])
            i += 1
        return barrier_number

    # def find_nearObstacle(self, node, obstacleList):
    #
    #     barrier_number = len(obstacleList)
    #     for i in range(barrier_number):
    #         dis = ((node.x - obstacleList[i].x) ** 2
    #                 +(node.y-obstacleList[i].y) ** 2
    #
    #                )
    #
    #
    #
    #     return list

    #######################
    # 关于采样点阈值；
    # 想法是：通过搜索node_list这个列表中节点之间的距离，来决定阈值P的值；
    # 经测试后：不符合预期，列表中的距离不断被累加，
    #######################
    @staticmethod
    def sample_rate(length):
        number = length / 3
        if 0 <= length < number:
            P = 10
        elif number <= length < 2 * length:
            P = 20
        elif 2 * number <= length <= 3 * length:
            P = 30
        else:
            P = 40
        return P

    #######################
    # 新采样函数：加入了一个以中点为圆心的采样范围
    #######################
    def new_sample(self, rate):
        p_random = random.randint(0, 100)
        if p_random > rate:
            rnd = [random.uniform(self.min_rand, self.max_rand), random.uniform(self.min_rand, self.max_rand)]
        elif rate < p_random < 2 * rate:
            rnd = self.circle_sample(self.get_middle_point(self.start, self.goal), 10)
        else:  # goal point sampling
            rnd = [self.goal.x, self.goal.y]
        return rnd

    #######################
    # 旧采样函数
    #######################
    def sample(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = [random.uniform(self.min_rand, self.max_rand), random.uniform(self.min_rand, self.max_rand)]
        else:  # goal point sampling
            rnd = [self.goal.x, self.goal.y]
        return rnd

    #######################
    # 在椭圆内采样
    #######################
    def informed_sample(self, cMax, cMin, xCenter, C):
        if cMax < float('inf'):
            r = [cMax / 2.0,
                 math.sqrt(cMax ** 2 - cMin ** 2) / 2.0,
                 math.sqrt(cMax ** 2 - cMin ** 2) / 2.0]
            L = np.diag(r)
            xBall = self.sample_unit_ball()  # 在单位圆中采样
            rnd = np.dot(np.dot(C, L), xBall) + xCenter  # 坐标平移变换后，将单位圆压缩成椭圆了
            rnd = [rnd[(0, 0)], rnd[(1, 0)]]
        else:
            rnd = self.sample()

        return rnd

    #######################
    # 重新选择父节点
    #######################
    def choose_parent(self, newNode, nearInds):
        if len(nearInds) == 0:  # 送过来的最近节点的下标列表，如果这个列表的长度为0，说明红圈内没有候选节点，直接返回这个节点，是最近的节点
            return newNode

        dList = []
        for i in nearInds:
            dx = newNode.x - self.node_list[i].x
            dy = newNode.y - self.node_list[i].y
            d = math.hypot(dx, dy)
            theta = math.atan2(dy, dx)
            if self.check_collision(self.node_list[i], theta, d):
                dList.append(self.node_list[i].cost + d)
            else:
                dList.append(float('inf'))  # 有碰撞，则记录无穷大

        minCost = min(dList)
        minInd = nearInds[dList.index(minCost)]

        if minCost == float('inf'):
            print("min cost is inf")
            return newNode

        newNode.cost = minCost
        newNode.parent = minInd

        return newNode

    #######################
    # 找到最近节点
    #######################
    def find_near_nodes(self, newNode):
        n_node = len(self.node_list)  # 记录整个节点列表长度
        r = 50.0 * math.sqrt((math.log(n_node) / n_node))
        d_list = [(node.x - newNode.x) ** 2 + (node.y - newNode.y) ** 2
                  for node in self.node_list]
        near_inds = [d_list.index(i) for i in d_list if i <= r ** 2]
        return near_inds  # 返回记录红圈内找到节点的列表

    @staticmethod
    def sample_unit_ball():
        a = random.random()
        b = random.random()

        if b < a:
            a, b = b, a

        sample = (b * math.cos(2 * math.pi * a / b),
                  b * math.sin(2 * math.pi * a / b))
        return np.array([[sample[0]], [sample[1]], [0]])

    #######################
    # 得到路径的长度
    #######################
    @staticmethod
    def get_path_len1(path):
        pathLen = 0
        for i in range(1, len(path)):  # 列表长度
            node1_x = path[i][0]
            node1_y = path[i][1]
            node2_x = path[i - 1][0]
            node2_y = path[i - 1][1]
            pathLen += math.sqrt((node1_x - node2_x)
                                 ** 2 + (node1_y - node2_y) ** 2)

        return pathLen

    #######################
    # 得到路径的长度
    #######################
    @staticmethod
    def get_path_len2(path):
        pathLen = 0
        for i in range(1, len(path)):  # 列表长度
            node1_x = path[i].x
            node1_y = path[i].y
            node2_x = path[i - 1].x
            node2_y = path[i - 1].y
            pathLen += math.sqrt((node1_x - node2_x)
                                 ** 2 + (node1_y - node2_y) ** 2)

        return pathLen

    #######################
    # 起始点与目标点直线距离
    #######################
    @staticmethod
    def line_cost(node1, node2):
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    #######################
    # 获得起始点和目标点连线的中点
    #######################
    @staticmethod
    def get_middle_point(point1, point2):
        middle_point_x = (point1.x + point2.x) / 2
        middle_point_y = (point1.y + point2.y) / 2
        middle_point = [middle_point_x, middle_point_y]
        return middle_point

    #######################
    # 遍历当前所有的节点，找到离采样点最近的节点，在这里传入的nodes是self.node_list,rnd是随机点
    #######################
    @staticmethod
    def get_nearest_list_index(nodes, rnd):
        dList = [(node.x - rnd[0]) ** 2
                 + (node.y - rnd[1]) ** 2 for node in nodes]
        minIndex = dList.index(min(dList))
        return minIndex  # 得到最近节点的下标

    #######################
    # 生成新的节点
    #######################
    def get_new_node(self, theta, n_ind, nearestNode):
        newNode = copy.deepcopy(nearestNode)

        newNode.x += self.expand_dis * math.cos(theta)
        newNode.y += self.expand_dis * math.sin(theta)

        newNode.cost += self.expand_dis
        newNode.parent = n_ind
        return newNode

    #######################
    # 论文中目标引力与动态步长法；qrand是随机采样点位置，随机树中与qrand最近的节点位置
    #######################
    def new_method_get_newNode(self, qrand_x, grand_y, qnearestNode, qgoal, k, newNode=None):

        rand_nearest_distance = math.sqrt((qrand_x - qnearestNode.x) ** 2 + (grand_y - qnearestNode.y) ** 2)
        goal_nearest_distance = math.sqrt((qgoal.x - qnearestNode.x) ** 2 + (qgoal.y - qnearestNode.y) ** 2)
        newNode.x = qnearestNode.x + (self.p1 * ((qrand_x - qnearestNode.x) / rand_nearest_distance)) \
                    + k * (self.p2 * ((qgoal.x - qnearestNode.x) / goal_nearest_distance))
        newNode.y = qnearestNode.y + (self.p1 * ((grand_y - qnearestNode.y) / rand_nearest_distance)) \
                    + k * (self.p2 * ((qgoal.y - qnearestNode.y) / goal_nearest_distance))

        # rand_nearest_distance = math.sqrt((qrand.x-qnearestNode.x)**2 + (qrand.y-qnearestNode.y)**2)
        # goal_nearest_distance = math.sqrt((qgoal.x-qnearestNode.x)**2+(qgoal.y-qnearestNode.y)**2)
        # newNode.x = qnearestNode.x + (self.p1 * ((qrand.x-qnearestNode.x) / rand_nearest_distance)) \
        #             + k * (self.p2 * ((qgoal.x-qnearestNode.x) / goal_nearest_distance))
        # newNode.y = qnearestNode.y + (self.p1 * ((qrand.y-qnearestNode.y) / rand_nearest_distance)) \
        #             + k * (self.p2 * ((qgoal.y-qnearestNode.y) / goal_nearest_distance))
        return newNode

    #######################
    # 检测是不是到了终点
    #######################
    def is_near_goal(self, node):
        d = self.line_cost(node, self.goal)
        if d < self.expand_dis:
            return True
        return False

    #######################
    # 重新连接节点
    #######################
    def rewire(self, newNode, nearInds):
        n_node = len(self.node_list)
        for i in nearInds:
            nearNode = self.node_list[i]

            d = math.sqrt((nearNode.x - newNode.x) ** 2
                          + (nearNode.y - newNode.y) ** 2)

            s_cost = newNode.cost + d

            if nearNode.cost > s_cost:
                theta = math.atan2(newNode.y - nearNode.y,
                                   newNode.x - nearNode.x)
                if self.check_collision(nearNode, theta, d):
                    nearNode.parent = n_node - 1
                    nearNode.cost = s_cost

    #######################

    #######################
    @staticmethod
    def distance_squared_point_to_segment(v, w, p):
        # Return minimum distance between line segment vw and point p
        if np.array_equal(v, w):
            return (p - v).dot(p - v)  # v == w case
        l2 = (w - v).dot(w - v)  # i.e. |w-v|^2 -  avoid a sqrt
        # Consider the line extending the segment,
        # parameterized as v + t (w - v).
        # We find projection of point p onto the line.
        # It falls where t = [(p-v) . (w-v)] / |w-v|^2
        # We clamp t from [0,1] to handle points outside the segment vw.
        t = max(0, min(1, (p - v).dot(w - v) / l2))
        projection = v + t * (w - v)  # Projection falls on the segment
        return (p - projection).dot(p - projection)

    #######################
    # 与check_collision() 配套使用，返回True或者False
    #######################
    def check_segment_collision_flag(self, x1, y1, x2, y2):
        for (ox, oy, size) in self.obstacle_list:
            dd = self.distance_squared_point_to_segment(
                np.array([x1, y1]),
                np.array([x2, y2]),
                np.array([ox, oy]))
            if dd <= size ** 2:
                return False  # collision
        return True

    #######################
    # 检测是否发生碰撞，与check_segment_collision_flag() 配套使用，返回碰撞检测标志
    #######################
    def check_collision(self, nearNode, theta, d):
        tmpNode = copy.deepcopy(nearNode)
        end_x = tmpNode.x + math.cos(theta) * d
        end_y = tmpNode.y + math.sin(theta) * d
        return self.check_segment_collision_flag(tmpNode.x, tmpNode.y, end_x, end_y)

    def get_final_course(self, lastIndex):
        path = [[self.goal.x, self.goal.y]]  # 列表中放入终点
        while self.node_list[lastIndex].parent is not None:  # 每个节点都是有个parent属性，起点设置是None
            # 循环到
            node = self.node_list[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])  # 这里
        return path

    #######################

    #######################
    @staticmethod
    def half_path(pathlength, path):
        halfcost = pathlength / 2

        pathLen = 0
        middle_node = None
        num = len(path)
        # re_path=path.reverse()
        # for i in range(1,len(re_path))
        print("路程中点的值%f,路径节点个数%d,中间最近节点是%s" % (halfcost, num, middle_node))
        for i in range(1, len(path)):
            if pathLen < halfcost:
                node1_x = path[i][0]
                node1_y = path[i][1]
                node2_x = path[i - 1][0]
                node2_y = path[i - 1][1]
                pathLen += math.sqrt((node1_x - node2_x)
                                     ** 2 + (node1_y - node2_y) ** 2)
                halfIndex = i
                middle_node = path[i]
                print("pathLen的长度", pathLen)
                print("下标", halfIndex)

        return middle_node

    #######################
    # 找到起始点和目标点之间的中点
    #######################
    @staticmethod
    def find_point_to_point_middle(point1, point2):
        node_x = (point1[0] + point2[0]) / 2
        node_y = (point1[1] + point2[1]) / 2
        node = [node_x, node_y]
        return node

    #######################
    # 用来寻找算法最开始的路径
    # 重要！！！！
    #######################
    def build_first_path(self, num, tempPathLen, tempPath, animation=False):
        start_time = time.time()
        path = None
        for i in range(self.max_iter):
            num += 1

            random_point = self.new_sample(20)
            # random_point = self.sample()
            near_point_index = self.get_nearest_list_index(self.node_list, random_point)
            nearestNode = self.node_list[near_point_index]
            two_point_theta = math.atan2(random_point[1] - nearestNode.y, random_point[0] - nearestNode.x)
            # 新方法newNode = self.new_method_get_newNode(random_point[0], random_point[1], nearestNode, self.goal, k)
            newNode = self.get_new_node(two_point_theta, near_point_index, nearestNode)
            noCollision_flag = self.check_segment_collision_flag(newNode.x, newNode.y, nearestNode.x, nearestNode.y)
            if noCollision_flag:
                nearInds_list = self.find_near_nodes(newNode)  # 寻找红圈内其他的节点，进行比较，这里返回的nearInds是一个最近节点的列表
                newNode = self.choose_parent(newNode, nearInds_list)  # 重新选择节点，传入nearInds这个列表

                self.node_list.append(newNode)
                self.rewire(newNode, nearInds_list)  # 传入红圈内节点的列表
                # current_path = self.get_path_len2(self.node_list)
                # print("长度",current_path)

                # for i in range (len(self.node_list)):
                #     print("当前节点",self.node_list[i].x,self.node_list[i].y)

                if animation:
                    self.draw_graph(newNode, path)

                if self.is_near_goal(newNode):  # 判断是否在终点附近
                    if self.check_segment_collision_flag(newNode.x, newNode.y,
                                                         self.goal.x, self.goal.y):  # 判断这个与终点的连线是否发生碰撞
                        lastIndex = len(self.node_list) - 1

                        tempPath = self.get_final_course(lastIndex)  # 找到路径
                        tempPathLen = self.get_path_len1(tempPath)

                        if self.path_success_flag(tempPath, self.start):
                            print("current path length: {}, It costs {} s, 迭代次数 {}".format(tempPathLen,
                                                                                           time.time() - start_time,
                                                                                           num))
                            break

        # 此处报错：NoneType has no len(),猜测有可能是寻找到了self.goal这个点
        # for i in range(0,len(tempPath)):
        #     print("",tempPath[i][0],tempPath[i][1])

        return [tempPath, tempPathLen, time.time() - start_time, num]

    #######################
    # 检测路径是否寻找成功
    #######################
    @staticmethod
    def path_success_flag(path, node):
        # print("00000",node.x,node.y)
        for i in range(1, len(path)):
            if path[i][0] == node.x:
                if path[i][1] == node.y:
                    return True
        return False

    #######################
    # 绘制普通采样图
    #######################
    def draw_graph(self, rnd=None, path=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")

        for node in self.node_list:
            if node.parent is not None:
                if node.x or node.y is not None:
                    plt.plot([node.x, self.node_list[node.parent].x], [
                        node.y, self.node_list[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacle_list:
            # self.plot_circle(ox, oy, size)
            plt.plot(ox, oy, "ok", ms=5 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")

        if path is not None:
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')

        plt.axis([-10, 40, -10, 40])
        # plt.axis([-2, 18, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    #######################
    # 绘制椭圆采样图
    #######################
    def draw_graph_informed_RRTStar(self, xCenter=None, cBest=None, cMin=None, e_theta=None, rnd=None, path=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
            if cBest != float('inf'):
                self.plot_ellipse(xCenter, cBest, cMin, e_theta)

        for node in self.node_list:
            if node.parent is not None:
                if node.x or node.y is not None:
                    plt.plot([node.x, self.node_list[node.parent].x], [
                        node.y, self.node_list[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacle_list:
            plt.plot(ox, oy, "ok", ms=5 * size)

        if path is not None:
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")

        plt.axis([-10, 40, -10, 40])
        # plt.axis([-2, 18, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    #######################

    #######################
    @staticmethod
    def plot_ellipse(xCenter, cBest, cMin, e_theta):  # pragma: no cover

        a = math.sqrt(cBest ** 2 - cMin ** 2) / 2.0
        b = cBest / 2.0
        angle = math.pi / 2.0 - e_theta
        cx = xCenter[0]
        cy = xCenter[1]
        t = np.arange(0, 2 * math.pi + 0.1, 0.1)
        x = [a * math.cos(it) for it in t]
        y = [b * math.sin(it) for it in t]
        rot = Rot.from_euler('z', -angle).as_matrix()[0:2, 0:2]
        fx = rot @ np.array([x, y])
        px = np.array(fx[0, :] + cx).flatten()
        py = np.array(fx[1, :] + cy).flatten()
        plt.plot(cx, cy, "xc")
        plt.plot(px, py, "--c")

    def informed_rrt_star_planning(self, start, goal, animation=True):
        start_time = time.time()
        # self.start = start
        # self.goal = goal
        path = None
        self.start = common.Node(start[0], start[1])
        self.goal = common.Node(goal[0], goal[1])
        self.node_list = [self.start]
        # for i in range(1,len(self.node_list)):
        #     print("测试：记录node_list中的初始值", self.node_list[i])
        path_number = 0

        # middle_node = self.find_point_to_point_middle(start, goal)

        cBest = float('inf')  # 将当前已经搜索到的最短路径作为cBest

        # # Computing the sampling space
        # cMin = math.sqrt(pow(self.start.x - self.goal.x, 2)
        #                  + pow(self.start.y - self.goal.y, 2))
        # xCenter = np.array([
        #                     [(self.start.x + self.goal.x) / 2.0],
        #                     [(self.start.y + self.goal.y) / 2.0],
        #                     [0]
        #                     ])   # 写成3维的形式
        # a1 = np.array([
        #                 [(self.goal.x - self.start.x) / cMin],
        #                 [(self.goal.y - self.start.y) / cMin],
        #                 [0]
        #                 ])
        #
        # e_theta = math.atan2(a1[1], a1[0])
        #
        # # 论文方法求旋转矩阵（2选1）
        # # first column of identity matrix transposed
        # # id1_t = np.array([1.0, 0.0, 0.0]).reshape(1, 3)
        # # M = a1 @ id1_t
        # # U, S, Vh = np.linalg.svd(M, True, True)
        # # C = np.dot(np.dot(U, np.diag(
        # #     [1.0, 1.0, np.linalg.det(U) * np.linalg.det(np.transpose(Vh))])),
        # #            Vh)
        #
        # 直接用二维平面上的公式（2选1）

        # C = np.array([[math.cos(self.theta_cal()), -math.sin(self.theta_cal()), 0],
        #               [math.sin(self.theta_cal()), math.cos(self.theta_cal()), 0],
        #               [0, 0, 1]])

        M = calculate.Calculate(start, goal)  # 调用类要实例化，实例化这个类 ,注意下这个start goal的参数
        ctheta = M.theta_common_cal()
        # cmin = M.cmin_common_cal()
        # xcenter = M.xcenter_common_cal()
        # print("打印角度值", ctheta)

        C1 = np.array([[math.cos(M.theta_common_cal()),
                        -math.sin(M.theta_common_cal()), 0],
                       [math.sin(M.theta_common_cal()),
                        math.cos(M.theta_common_cal()), 0],
                       [0, 0, 1]])

        #######################
        # 用来测试第一次找路径，测试ok，同时用来生成新、旧采样方法.xlsx
        # num = 0
        # length = 0
        # path = None
        # first_path = self.build_first_path(num, length, path)
        #######################

        # mi_point = self.get_middle_point(self.start, self.goal)
        # print("中间点",mi_point)
        # point = self.circle_sample(mi_point, 10)
        # print("圆内随机点",point)

        line_distance = self.line_cost(self.start, self.goal)
        print("直线距离", line_distance)

        for i in range(self.max_iter):

            # --------------------------------------

            # 最原始 rnd = self.informed_sample(cBest, cMin, xCenter, C)  # 针对随机点的采样进行了修改
            # rnd = self.informed_sample(cBest, self.cmin_cal(), self.xcenter_cal(self.cmin_cal()), C)
            # rnd = self.informed_sample(cBest, self.cmin_cal(), self.xcenter_cal(self.cmin_cal()), C1)
            # 将cmin,xcenter函数写在Calculate这个类中  rnd = self.informed_sample(cBest, cmin, xcenter, C1)

            # self.run_game(cBest,C1,ctheta) 有错误

            # ---------------------------------------------

            rnd = self.informed_sample(cBest, calculate.Calculate.cmin_common_cal(self.start, self.goal),
                                       calculate.Calculate.xcenter_common_cal(self.start, self.goal), C1)
            n_ind = self.get_nearest_list_index(self.node_list, rnd)  # 找到离采样点最近的节点的下标
            nearestNode = self.node_list[n_ind]

            # steer
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
            newNode = self.get_new_node(theta, n_ind, nearestNode)

            noCollision = self.check_segment_collision_flag(newNode.x, newNode.y, nearestNode.x, nearestNode.y)
            if noCollision:
                nearInds = self.find_near_nodes(newNode)  # 寻找红圈内其他的节点，进行比较，这里返回的nearInds是一个最近节点的列表
                newNode = self.choose_parent(newNode, nearInds)  # 重新选择节点，传入nearInds这个列表

                self.node_list.append(newNode)
                self.rewire(newNode, nearInds)  # 传入红圈内节点的列表

                if self.is_near_goal(newNode):  # 判断是否在终点附近
                    # 判断这个与终点的连线是否发生碰撞
                    if self.check_segment_collision_flag(newNode.x, newNode.y, self.goal.x, self.goal.y):
                        lastIndex = len(self.node_list) - 1
                        tempPath = self.get_final_course(lastIndex)  # 找到路径
                        tempPathLen = self.get_path_len1(tempPath)

                        if tempPathLen < cBest:
                            path = tempPath
                            cBest = tempPathLen
                            print(
                                "current path length: {}, It costs {} s".format(tempPathLen,
                                                                                time.time() - start_time))
                            path_number += 1
                            print("第%d次找到路径" % path_number)
                            print("cBest的值%f" % cBest)
                            for y in range(0, len(path)):
                                print("x轴为%f,y轴为%f" % (path[i][0], path[i][1]))
                            self.half_path(cBest, path)

            if animation:
                self.draw_graph_informed_RRTStar(
                    xCenter=calculate.Calculate.xcenter_common_cal(self.start, self.goal),
                    cBest=cBest, cMin=calculate.Calculate.cmin_common_cal(self.start, self.goal),
                    e_theta=M.theta_common_cal(), rnd=rnd, path=path)
                # 这里是采用实例化calculate.Calculate，与上面M = calculate.Calculate(start, goal)是一起的
                # self.draw_graph_informed_RRTStar(xCenter=xcenter,
                #                                  cBest=cBest, cMin=cmin,
                #                                  e_theta=ctheta, rnd=rnd, path=path)

                # 这2行是显示进行多少次循环了
                # print("第 %d 次迭代" %i)
                # self.t += 1

        return path
