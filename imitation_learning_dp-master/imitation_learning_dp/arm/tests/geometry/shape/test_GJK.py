import numpy as np
from spatialmath import SE3
from arm.geometry import LineSegment, Point, GJK, Triangle, Tetrahedron, Brick


def test_calculate():
    triangle0 = Triangle([Point([0, 0, 0]), Point([1, 1, 0]), Point([2, 0, 0])])
    triangle1 = Triangle([Point([0, -3, 0]), Point([4, 1, 0]), Point([5, 0, 0])])
    distance = GJK.calculate_distance(triangle0, triangle1)
    print('distance: ', distance)


def test_calculate_nearest_point_to_line_segment():
    point = Point([0, 1, 0])
    line_segment = LineSegment(Point([0, 0, 0]), Point([1, 1, 0]))
    point_to_line_segment = GJK.calculate_closest_point_to_line_segment(point, line_segment)
    print('point_to_line_segment: ', point_to_line_segment)


def test_calculate_nearest_point_to_triangle():
    point = Point([1, 0.5, 0])
    triangle = Triangle([Point([0, 0, 0]), Point([1, 1, 0]), Point([2, 0, 0])])
    point_to_triangle = GJK.calculate_closest_point_to_triangle(point, triangle)
    print('point_to_triangle: ', point_to_triangle)


def test_calculate_closest_point_to_tetrahedron():
    point = Point([-1, 0.5, 0.5])
    tetrahedron = Tetrahedron([Point([0, 0, 0]), Point([1, 0, 0]), Point([0, 1, 0]), Point([0, 0, 1])])
    point_to_tetrahedron = GJK.calculate_closest_point_to_tetrahedron(point, tetrahedron)
    print('point_to_tetrahedron: ', point_to_tetrahedron)


def test_brick_to_brick():
    brick0 = Brick(SE3(), np.array([1, 1, 1]))
    brick1 = Brick(SE3.Trans(2.0, 2.0, 5), np.array([1, 1, 1]))
    calculate = GJK.calculate_distance(brick0, brick1)
    print('calculate: ', calculate)


def test_brick_to_tetrahedron():
    brick = Brick(SE3().Trans(2, 2, 0), np.array([1, 1, 1]))
    tetrahedron = Tetrahedron([Point([0, 0, 0]), Point([1, 0, 0]), Point([0, 1, 0]), Point([0, 0, 1])])
    calculate = GJK.calculate_distance(brick, tetrahedron)
    print('calculate: ', calculate)


def test_point_to_brick():
    point = Point(np.array([0, 0, 3]))
    brick = Brick(SE3(), np.array([1, 1, 1]))
    calculate = GJK.calculate_distance(point, brick)
    print('calculate: ', calculate)


def test_line_segment_to_brick():
    point0 = Point(np.array([0, 0, 2]))
    point1 = Point(np.array([1, 1, 2]))
    line_segment = LineSegment(point0, point1)
    brick = Brick(SE3(), np.array([1, 1, 1]))
    calculate = GJK.calculate_distance(line_segment, brick)
    print('calculate: ', calculate)


def test_line_segment_to_brick2():
    point0 = Point(np.array([0, 0, 2]))
    point1 = Point(np.array([1, 1, 2]))
    line_segment = LineSegment(point0, point1)
    brick = Brick(SE3(), np.array([1, 1, 1]))
    calculate, points = GJK.calculate_distance_and_points(line_segment, brick)
    print('calculate: ', calculate)
    print('point0: ', points[0])
    print('point1: ', points[1])


if __name__ == '__main__':
    # pass
    test_calculate()
    test_brick_to_brick()
    test_brick_to_tetrahedron()
    test_point_to_brick()
    test_line_segment_to_brick()
    test_line_segment_to_brick2()
