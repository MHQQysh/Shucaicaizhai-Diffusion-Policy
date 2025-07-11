import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from spatialmath import SE3

import numpy as np
from spatialmath import SE3

from arm.geometry import Sphere

if __name__ == '__main__':
    sphere = Sphere(SE3(), 0.5)

    # 创建一个图形对象和一个三维坐标轴
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    sphere.plot(ax)

    # 设置坐标轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])

    # 显示图形
    plt.show()