from unittest import TestCase
from arm.geometry import LineSegment, Point, Plane, SE3Impl, Brick, Distance, Cylinder, Sphere
import numpy as np
from spatialmath import SE3


class TestLineSegment(TestCase):
    def test_calculate_barycentric_coordinates(self):
        p0 = Point([-1, 0, 0.0])
        p1 = Point([1, 0, 0.0])
        line_segment = LineSegment(p0, p1)

        p2 = Point([0, 0, 0.0])
        coordinates = line_segment.calculate_barycentric_coordinates(p2)
        print('coordinates: ', coordinates)
