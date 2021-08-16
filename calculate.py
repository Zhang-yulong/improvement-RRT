import math
import common
import numpy as np


class Calculate:

    def __init__(self, node1, node2):

        self.node1 = common.Node(node1[0], node1[1])
        self.node2 = common.Node(node2[0], node2[1])
        # print("调用Calculate构造函数")

    # def cmin_cal(self):
    #     cMin = math.sqrt(pow(self.start.x - self.goal.x, 2)
    #                          + pow(self.start.y - self.goal.y, 2))
    #     return cMin

    # def xcenter_cal(self, cMin):
    #     xCenter = np.array([
    #         [(self.start.x + self.goal.x) / 2.0],
    #         [(self.start.y + self.goal.y) / 2.0],
    #         [0]])  # 写成3维的形式
    #
    #     return xCenter

    # def theta_cal(self):
    #     a1 = np.array([
    #         [(self.goal.x - self.start.x) / self.cmin_cal()],
    #         [(self.goal.y - self.start.y) / self.cmin_cal()],
    #         [0]])
    #     e_theta = math.atan2(a1[1], a1[0])
    #
    #     return e_theta




    # def cmin_common_cal(self):
    #
    #     cMin = math.sqrt(pow(self.node1.x - self.node2.x, 2)
    #                      + pow(self.node1.y - self.node2.y, 2))
    #     return cMin
    #
    # def xcenter_common_cal(self):
    #     xCenter = np.array([
    #         [(self.node1.x + self.node2.x) / 2.0],
    #         [(self.node1.y + self.node2.y) / 2.0],
    #         [0]])  # 写成3维的形式
    #
    #     return xCenter
    #
    def theta_common_cal(self):
        a1 = np.array([
            [(self.node2.x - self.node1.x) / self.cmin_common_cal(self.node1, self.node2)],
            [(self.node2.y - self.node1.y) / self.cmin_common_cal(self.node1, self.node2)],
            [0]])
        e_theta = math.atan2(a1[1], a1[0])

        return e_theta


    # ----------------------------想使用静态方法staticmethod

    @staticmethod
    def cmin_common_cal(node1, node2):  # 使用staticmethod，形参不能加self

        cMin = math.sqrt(pow(node1.x - node2.x, 2)
                         + pow(node1.y - node2.y, 2))
        return cMin

    @staticmethod
    def xcenter_common_cal(node1, node2):
        xCenter = np.array([
            [(node1.x + node2.x) / 2.0],
            [(node1.y + node2.y) / 2.0],
            [0]])  # 写成3维的形式

        return xCenter

    # @staticmethod
    # def theta_common_cal(node1, node2):
    #     M = Calculate(node1, node2)
    #     a1 = np.array([
    #         [(node2.x - node1.x) / M.cmin_common_cal(node1, node2)],
    #         [(node2.y - node1.y) / M.cmin_common_cal(node1, node2)],
    #         [0]])
    #     e_theta = math.atan2(a1[1], a1[0])
    #
    #     return e_theta