from unittest import TestCase

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
from spatialmath import SE3
from arm.geometry import Capsule, UnitVector


class TestCapsule(TestCase):
    def test_calculate_support_point(self):
        capsule = Capsule(SE3(), 1.0, 1.0)
        d = UnitVector(np.array([0, 0, 1.0]))
        point = capsule.calculate_support_point(d)
        print('point: ', point)

    def test_plot(self):
        capsule = Capsule(SE3(), 0.2, 1.0)
        fig = plt.figure(1)
        ax = plt.axes(projection='3d')
        capsule.plot(ax)

        ax.set_xlim([-2, 2])
        ax.set_ylim([-2, 2])
        ax.set_zlim([-2, 2])

        ax.set_box_aspect([1, 1, 1])
        plt.show()