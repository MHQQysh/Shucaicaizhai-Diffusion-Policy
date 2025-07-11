from unittest import TestCase

import numpy as np
from spatialmath import SE3
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from arm.geometry import Point, LineSegment, Brick, Sphere, Cylinder, Capsule, Distance, Collision


def plot(shape0, shape1):
    fig = plt.figure(1)
    ax = plt.axes(projection='3d')

    shape0.plot(ax)
    shape1.plot(ax)

    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])

    ax.set_box_aspect([1, 1, 1])

    plt.show()


class TestCollision(TestCase):
    def test_cylinder_and_sphere(self):
        cylinder = Cylinder(SE3(), 0.5, 0.5)
        sphere = Sphere(SE3(), 0.5)
        collision = Collision.is_collision(cylinder, sphere)
        print("collision", collision)
        plot(cylinder, sphere)

    def test_cylinder_and_sphere2(self):
        cylinder = Cylinder(SE3(), 0.5, 0.5)
        sphere = Sphere(SE3.Trans(1.2, 0, 0), 0.5)
        collision = Collision.is_collision(cylinder, sphere)
        print("collision", collision)
        plot(cylinder, sphere)

    def test_brick_and_sphere(self):
        brick = Brick(SE3(), np.array([0.5, 0.5, 0.5]))
        sphere = Sphere(SE3.Trans(0.76, 0, 0), 0.5)
        collision = Collision.is_collision(brick, sphere)
        print("collision", collision)
        plot(brick, sphere)

    def test_brick_and_cylinder(self):
        brick = Brick(SE3(), np.array([0.5, 0.5, 0.5]))
        cylinder = Cylinder(SE3.Trans(0.46, 0, 0), 0.2, 0.2)
        collision = Collision.is_collision(brick, cylinder)
        print("collision", collision)
        plot(brick, cylinder)

    def test_brick_and_brick(self):
        brick0 = Brick(SE3(), np.array([0.5, 0.5, 0.5]))
        brick1 = Brick(SE3.Trans(0.51, 0, 0), np.array([0.5, 0.5, 0.5]))
        collision = Collision.is_collision(brick0, brick1)
        print("collision", collision)
        plot(brick0, brick1)

    def test_brick_and_capsule(self):
        brick = Brick(SE3(), np.array([0.5, 0.5, 0.5]))
        capsule = Capsule(SE3.Trans(0.451, 0, 0), 0.2, 1.0)
        collision = Collision.is_collision(brick, capsule)
        print("collision", collision)
        plot(brick, capsule)
