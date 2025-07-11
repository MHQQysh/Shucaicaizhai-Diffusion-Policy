import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
from spatialmath import SE3

from arm.geometry import Brick

if __name__ == '__main__':
    brick = Brick(SE3.Rx(np.pi/4), np.array([1, 2, 3]))
    xn_yn_zn_point = brick.xn_yn_zn_point
    print('xn_yn_zn_point: ', xn_yn_zn_point)
    xp_yn_zn_point = brick.xp_yn_zn_point
    print('xp_yn_zn_point: ', xp_yn_zn_point)
    xn_yp_zn_point = brick.xn_yp_zn_point
    print('xn_yp_zn_point: ', xn_yp_zn_point)
    xp_yp_zn_point = brick.xp_yp_zn_point
    print('xp_yp_zn_point: ', xp_yp_zn_point)

    xn_yn_zp_point = brick.xn_yn_zp_point
    print('xn_yn_zp_point: ', xn_yn_zp_point)
    xp_yn_zp_point = brick.xp_yn_zp_point
    print('xp_yn_zp_point: ', xp_yn_zp_point)
    xn_yp_zp_point = brick.xn_yp_zp_point
    print('xn_yp_zp_point: ', xn_yp_zp_point)
    xp_yp_zp_point = brick.xp_yp_zp_point
    print('xp_yp_zp_point: ', xp_yp_zp_point)

    print('points: ', brick.points)
    for point in brick.points:
        print(point)
    print('****************')

    # 创建一个图形对象和一个三维坐标轴
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    brick.plot(ax)

    # 设置坐标轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])

    # 显示图形
    plt.show()
