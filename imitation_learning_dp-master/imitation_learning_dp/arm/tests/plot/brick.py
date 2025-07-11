import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

if __name__ == '__main__':
    # 创建一个图形对象和一个三维坐标轴
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 立方体参数
    length = 1
    width = 1
    height = 1

    # 生成立方体的顶点坐标
    x = [0, length, length, 0, 0, length, length, 0]
    y = [0, 0, width, width, 0, 0, width, width]
    z = [0, 0, 0, 0, height, height, height, height]

    # 将 Z 数组转换为二维数组
    Z = np.array(z).reshape(2, 4)

    # 将参数 0 的形状扩展为 (2, 4)
    X = np.array(x).reshape(2, 4)
    Y = np.array(y).reshape(2, 4)

    # 绘制立方体的六个面
    ax.plot_surface(X[:, :2], Y[:, :2], Z[:, :2], alpha=0.5)  # 前面
    # ax.plot_surface(X[:, 1:3], Y[:, 1:3], Z[:, 1:3], alpha=0.5)  # 右面
    # ax.plot_surface(X[:, 2:], Y[:, 2:], Z[:, 2:], alpha=0.5)  # 后面
    # ax.plot_surface(X[:, [0, 3]], Y[:, [0, 3]], Z[:, [0, 3]], alpha=0.5)  # 左面
    # ax.plot_surface(X[1:3, :], Y[1:3, :], Z[1:3, :], alpha=0.5)  # 上面
    # ax.plot_surface(X[2:4, :], Y[2:4, :], Z[2:4, :], alpha=0.5)  # 下面

    # 设置坐标轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])

    # 显示图形
    plt.show()