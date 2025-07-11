import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

if __name__ == '__main__':

    # 创建一个图形对象和一个三维坐标轴
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 球体参数
    r = 1
    phi, theta = np.mgrid[0:np.pi:50j, 0:2*np.pi:50j]
    x = r*np.sin(phi)*np.cos(theta)
    y = r*np.sin(phi)*np.sin(theta)
    z = r*np.cos(phi)

    # 绘制球体
    ax.plot_surface(x, y, z, color='b', alpha=0.5)

    # 设置坐标轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # 显示图形
    plt.show()