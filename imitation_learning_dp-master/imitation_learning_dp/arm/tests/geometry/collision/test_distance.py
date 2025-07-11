from unittest import TestCase

import numpy as np
from spatialmath import SE3
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from arm.geometry import Point, LineSegment, Brick, Sphere, Cylinder, Capsule, Distance


def plot(shape0, shape1):
    distance, points = Distance.calculate_distance_and_points(shape0, shape1)

    plt.clf()
    fig = plt.figure(1)
    ax = plt.axes(projection='3d')

    shape0.plot(ax)
    shape1.plot(ax)
    for point in points:
        point.plot(ax)
    line_segment = LineSegment(points)
    line_segment.plot(ax)

    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])

    ax.set_box_aspect([1, 1, 1])

    plt.pause(0.1)


class TestDistance(TestCase):
    def test_cylinder_and_sphere(self):
        thetas = np.linspace(0, 2 * np.pi, 100)
        for i, theta in enumerate(thetas):
            cylinder = Cylinder(SE3(), 0.5, 0.5)
            sphere = Sphere(SE3.Trans(2.0 * np.sin(theta), 2.0 * np.cos(theta), 2.0 * np.sin(theta)), 0.5)
            plot(cylinder, sphere)
        plt.show()

    def test_brick_and_sphere(self):
        thetas = np.linspace(0, 2 * np.pi, 100)
        for i, theta in enumerate(thetas):
            brick = Brick(SE3.Rx(theta), np.array([0.5, 0.5, 0.5]))
            sphere = Sphere(SE3.Trans(2.0 * np.sin(theta), 2.0 * np.cos(theta), 2.0 * np.sin(theta)), 0.5)
            plot(brick, sphere)
        plt.show()

    def test_brick_and_cylinder(self):
        thetas = np.linspace(0, 2 * np.pi, 100)
        for i, theta in enumerate(thetas):
            brick = Brick(SE3.Rx(theta), np.array([0.5, 0.5, 0.5]))
            cylinder = Cylinder(
                SE3.Trans(1.0 * np.sin(theta), 1.0 * np.cos(theta), 1.0 * np.sin(theta)) * SE3.Rz(theta) * SE3.Ry(
                    theta), 0.2, 0.2)
            plot(brick, cylinder)
        plt.show()

    def test_brick_and_brick(self):
        thetas = np.linspace(0, 2 * np.pi, 100)
        for i, theta in enumerate(thetas):
            brick0 = Brick(SE3.Rx(theta), np.array([0.5, 0.5, 0.5]))
            brick1 = Brick(SE3.Trans(1.0 * np.sin(theta), 1.0 * np.cos(theta), 0.0) * SE3.Rz(theta) * SE3.Ry(theta),
                           np.array([0.5, 0.5, 0.5]))

            plot(brick0, brick1)
        plt.show()

    def test_brick_and_capsule(self):
        thetas = np.linspace(0, 2 * np.pi, 100)
        for i, theta in enumerate(thetas):
            brick = Brick(SE3.Rx(theta), np.array([0.5, 0.5, 0.5]))
            capsule = Capsule(
                SE3.Trans(1.0 * np.sin(theta), 1.0 * np.cos(theta), 1.0 * np.sin(theta)) * SE3.Rz(theta) * SE3.Ry(
                    theta), 0.2, 1.0)
            plot(brick, capsule)
        plt.show()
