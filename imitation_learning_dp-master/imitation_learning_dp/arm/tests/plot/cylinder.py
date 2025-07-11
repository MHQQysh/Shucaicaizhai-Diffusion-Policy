import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

if __name__ == '__main__':

    # 创建一个图形对象和一个三维坐标轴
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 圆柱体参数
    r = 1
    h = 2
    theta = np.linspace(0, 2*np.pi, 100)
    z = np.linspace(0, h, 20)
    theta, z = np.meshgrid(theta, z)
    x = r*np.cos(theta)
    y = r*np.sin(theta)

    # 绘制圆柱体
    ax.plot_surface(x, y, z, alpha=0.5)

    # 设置坐标轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # 显示图形
    plt.show()
