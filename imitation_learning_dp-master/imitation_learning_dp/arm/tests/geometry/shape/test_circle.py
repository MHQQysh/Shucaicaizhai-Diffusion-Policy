from unittest import TestCase

import numpy as np
from spatialmath import SE3
from arm.geometry import Circle2D, UnitVector


class TestCircle(TestCase):
    def test_calculate_support_point(self):
        circle = Circle2D(SE3.Trans(0.0, 0.0, 0.0), 1.0)
        d = UnitVector(np.array([0, 0, 1]))
        support_point = circle.calculate_support_point(d)
        print('support_point: ', support_point)
