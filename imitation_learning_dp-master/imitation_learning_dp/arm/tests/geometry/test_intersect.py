from unittest import TestCase

from arm.geometry import Point, Line, LineSegment, Circle2D, Intersect2D


class TestIntersect(TestCase):

    def test_check_point_to_circle(self):
        point = Point((1, 1))
        circle = Circle2D((0, 0), 1)
        intersect = Intersect2D.check_point_to_circle(point, circle)
        assert not intersect, "intersect"

    def test_check_line_to_circle(self):
        line = Line((0, 0), (0.5, 0))
        circle = Circle2D((1, 1), 0.5)

        intersect = Intersect2D.check_line_to_circle(line, circle)
        assert not intersect, "intersect"

    def test_check_line_segment_to_circle(self):
        line_segment = LineSegment((0, 0), (0.5, 0))
        circle = Circle2D((1, 1), 1)

        intersect = Intersect2D.check_line_segment_to_circle(line_segment, circle)
        assert not intersect, "intersect"