import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
    # 创建一个图形对象和一个三维坐标轴
    # 创建一个图形对象和一个三维坐标轴
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 定义圆面的参数化方程
    r = 1  # 半径
    phi, theta = np.mgrid[0:np.pi:50j, 0:2 * np.pi:50j]
    x = r * np.sin(phi) * np.cos(theta)
    y = r * np.sin(phi) * np.sin(theta)
    z = r * np.cos(phi) * 0

    # 绘制圆面
    ax.plot_surface(x, y, z, cmap='viridis')

    # 设置坐标轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # 显示图形
    plt.show()
