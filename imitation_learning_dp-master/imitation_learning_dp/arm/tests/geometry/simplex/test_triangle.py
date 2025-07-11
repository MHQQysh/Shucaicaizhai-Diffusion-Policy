from unittest import TestCase

import numpy as np

from arm.geometry import Triangle, Point


class TestTriangle(TestCase):
    def test_get_support(self):
        triangle0_point0 = Point(np.array([0, 0, 0]))
        triangle0_point1 = Point(np.array([2, 0, 0]))
        triangle0_point2 = Point(np.array([1, 2, 0]))
        triangle0_points = [triangle0_point0, triangle0_point1, triangle0_point2]

        triangle0 = Triangle(triangle0_points)
        
        d = np.array([1, 1, 0])
        support_point = triangle0.calculate_support_point(d)
        print('support point: ', support_point)

        # triangle1_point0 = Point(np.array([3, 0, 0]))
        # triangle1_point1 = Point(np.array([5, 0, 0]))
        # triangle1_point2 = Point(np.array([4, 1, 0]))
        # triangle1_points = [triangle1_point0, triangle1_point1, triangle1_point2]
        #
        # triangle1 = Triangle(triangle1_points)

    def test_calculate_barycentric_coordinates(self):
        triangle_point0 = Point(np.array([0, 0, 0]))
        triangle_point1 = Point(np.array([2, 0, 0]))
        triangle_point2 = Point(np.array([1, 2, 0]))
        triangle_points = [triangle_point0, triangle_point1, triangle_point2]

        triangle = Triangle(triangle_points)

        p2 = Point([1.0, 1.0, 0.0])
        coordinates = triangle.calculate_barycentric_coordinates(p2)
        print('coordinates: ', coordinates)
