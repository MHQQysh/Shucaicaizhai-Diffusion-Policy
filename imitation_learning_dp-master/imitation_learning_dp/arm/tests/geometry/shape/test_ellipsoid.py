from unittest import TestCase

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
from spatialmath import SE3
from arm.geometry import Capsule, UnitVector, Ellipsoid


class TestEllipsoid(TestCase):
    def test_plot(self):
        ellipsoid = Ellipsoid(SE3(), np.array([0.5, 0.1, 0.1]))
        fig = plt.figure(1)
        ax = plt.axes(projection='3d')
        ellipsoid.plot(ax)

        ax.set_xlim([-2, 2])
        ax.set_ylim([-2, 2])
        ax.set_zlim([-2, 2])

        ax.set_box_aspect([1, 1, 1])
        plt.show()
