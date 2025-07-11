from unittest import TestCase

import numpy as np
from spatialmath import SE3
from arm.geometry import Cylinder, GJK

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class TestCylinder(TestCase):
    def test_calculate_support_point(self):
        cylinder0 = Cylinder(SE3(), 1.0, 1.0)
        cylinder1 = Cylinder(SE3.Tz(2.0), 1.0, 1.0)
        calculate = GJK.calculate_distance(cylinder0, cylinder1)
        print('calculate: ', calculate)

    def test_plot(self):
        cylinder = Cylinder(SE3.Rx(np.pi/3), 1.0, 1.0)

        # 创建一个图形对象和一个三维坐标轴
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        cylinder.plot(ax)

        # 设置坐标轴标签
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.set_xlim([-2, 2])
        ax.set_ylim([-2, 2])
        ax.set_zlim([-2, 2])

        # 显示图形
        plt.show()
