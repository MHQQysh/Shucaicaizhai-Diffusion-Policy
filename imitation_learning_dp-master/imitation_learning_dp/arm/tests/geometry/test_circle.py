from unittest import TestCase

import numpy as np

from arm.geometry import Circle2D


class TestCircle(TestCase):
    def test_circle0(self):
        center = [2, 3]
        radius = 1.0
        circle0 = Circle2D(center, radius)
        print('circle0: ', circle0.get_center(), circle0.get_radius())

    def test_circle1(self):
        circle1 = Circle2D()
        print('circle1: ', circle1.get_center(), circle1.get_radius())
