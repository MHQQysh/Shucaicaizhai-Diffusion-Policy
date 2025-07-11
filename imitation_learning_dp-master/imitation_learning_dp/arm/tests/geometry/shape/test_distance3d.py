from unittest import TestCase
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
from spatialmath import SE3

from arm.geometry import LineSegment, Point, Plane, SE3Impl, Brick, Distance, Cylinder, Sphere


class TestDistance3D(TestCase):
    def test_point_to_point(self):
        p0 = Point([0.0, 0.0, 0.0])
        p1 = Point([1.0, 1.0, 1.0])
        distance = Distance.point_to_point(p0, p1)
        print('distance: ', distance)

    def test_point_to_plane(self):
        point = Point([0.0, 0.0, -1.0])
        plane = Plane(SE3Impl().Trans(-1, 0, 0).Rx(np.pi / 4))
        distance = Distance.point_to_plane(point, plane)
        print('distance: ', distance)

    def test_point_to_brick(self):
        point = Point([1.0, 1.0, 1.0])
        brick = Brick(SE3(), np.array([0.5, 0.5, 0.5]))
        d_point_to_brick = Distance.point_to_brick(point, brick)
        print('d_point_to_brick: ', d_point_to_brick)

    def test_line_to_line(self):
        p0 = Point([-1, 0, 0.0])
        p1 = Point([1, 0, 0.0])
        line_segment0 = LineSegment(p0, p1)

        q0 = Point([0, 1, 0.0])
        q1 = Point([0, 2, 0.0])
        line_segment1 = LineSegment(q0, q1)

        Distance.line_segment_to_line_segment(line_segment0, line_segment1)

    def test_plot(self):
        brick0 = Brick(SE3.Rx(np.pi / 12), np.array([0.5, 0.5, 0.5]))
        brick1 = Brick(SE3.Trans(1.0, 0.5, 0.2), np.array([0.5, 0.5, 0.5]))

        cylinder = Cylinder(SE3.Trans(-1, -1, 0), radius=0.2, length=0.5)

        sphere = Sphere(SE3.Trans(1.0, 0.2, 0), radius=0.2)

        p0 = Point([-1, 0, 0.0])
        p1 = Point([1, 0, 0.0])
        line_segment0 = LineSegment(p0, p1)

        q0 = Point([0, 1, 0.0])
        q1 = Point([0, 2, 0.0])
        line_segment1 = LineSegment(q0, q1)

        # distance, points = Distance.calculate_distance_and_points(brick0, brick1)

        distance1, points1 = Distance.calculate_distance_and_points(cylinder, brick0)
        #
        # distance2, points2 = Distance.calculate_distance_and_points(brick0, sphere)

        # distance3, points3 = Distance.calculate_distance_and_points(line_segment0, line_segment1)
        # print(distance2)

        # 创建一个图形对象和一个三维坐标轴
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        brick0.plot(ax)
        # brick1.plot(ax)
        # points[0].plot(ax)
        # points[1].plot(ax)

        cylinder.plot(ax)
        points1[0].plot(ax)
        points1[1].plot(ax)

        # brick0.plot(ax)
        # sphere.plot(ax)
        # points2[0].plot(ax)
        # points2[1].plot(ax)
        #
        # line_segment = LineSegment(points2)
        # line_segment.plot(ax)

        # cylinder.plot(ax)
        # distance3.plot(ax)
        # line_segment0.plot(ax)
        # line_segment1.plot(ax)
        # points3[0].plot(ax)
        # points3[1].plot(ax)
        # line_segment = LineSegment(points3)
        # line_segment.plot(ax)

        ax.set_xlim([-2, 2])
        ax.set_ylim([-2, 2])
        ax.set_zlim([-2, 2])

        ax.set_box_aspect([1, 1, 1])

        # 显示图形
        plt.show()
