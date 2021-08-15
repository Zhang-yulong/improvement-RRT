from matplotlib import pyplot as plt
import re_informedRRT
import common


show_animation = True


def main():
    print("Start rrt planning")

    obstacle1 = common.obstacle(3, 3, 3)
    obstacle2 = common.obstacle(5, 25, 5)
    obstacle3 = common.obstacle(9, 18, 4)
    obstacle4 = common.obstacle(12, 2, 2)
    obstacle5 = common.obstacle(16, 20, 3)
    obstacle6 = common.obstacle(22, 13, 5)
    obstacle7 = common.obstacle(27, 26, 2)

    obstacleList = [(obstacle1.x, obstacle1.y, obstacle1.r),
                    (obstacle2.x, obstacle2.y, obstacle2.r),
                    (obstacle3.x, obstacle3.y, obstacle3.r),
                    (obstacle4.x, obstacle4.y, obstacle4.r),
                    (obstacle5.x, obstacle5.y, obstacle5.r),
                    (obstacle6.x, obstacle6.y, obstacle6.r),
                    (obstacle7.x, obstacle7.y, obstacle7.r),
                    ]

    # a = RRT.show_obstacle(obstacleList)

    # create obstacles
    # obstacleList = [
    #     (3,  3,  1.5),
    #     (12, 2,  3),
    #     (3,  9,  2),
    #     (9,  11, 2),
    #     (12.5, 9, 1),
    # ]

    # Set params
    rrt = re_informedRRT.RRT(randArea=[-10, 40], obstacleList=obstacleList, maxIter=300)
    # print("打印obstacle的值", obstacleList)
    # path = rrt.rrt_planning(start=[0, 0], goal=[15, 12], animation=show_animation)
    # path = rrt.rrt_star_planning(start=[0, 0], goal=[15, 12], animation=show_animation)
    i = 0
    for i in range(21):
        path = rrt.informed_rrt_star_planning(start=[0, 0], goal=[35, 33], y=i, animation=show_animation)
        print("Done!!")
    path = None
    if show_animation and path:
        plt.show()


if __name__ == '__main__':
    main()
