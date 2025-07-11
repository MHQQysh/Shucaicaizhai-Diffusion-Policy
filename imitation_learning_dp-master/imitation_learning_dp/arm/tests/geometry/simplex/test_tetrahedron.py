from unittest import TestCase
import numpy as np

from arm.geometry import Triangle, Point, Tetrahedron


class TestTetrahedron(TestCase):
    def test_calculate_barycentric_coordinates(self):
        tetrahedron_point0 = Point(np.array([0, 0, 0]))
        tetrahedron_point1 = Point(np.array([2, 0, 0]))
        tetrahedron_point2 = Point(np.array([1, 2, 0]))
        tetrahedron_point3 = Point(np.array([0, 0, 1]))
        tetrahedron_points = [tetrahedron_point0, tetrahedron_point1, tetrahedron_point2, tetrahedron_point3]

        tetrahedron = Tetrahedron(tetrahedron_points)

        point = Point(np.array([0, 0, 0.5]))
        coordinates = tetrahedron.calculate_barycentric_coordinates(point)
        print('coordinates: ', coordinates)
