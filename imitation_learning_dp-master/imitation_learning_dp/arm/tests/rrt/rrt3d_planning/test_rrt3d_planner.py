from unittest import TestCase

import numpy as np
from spatialmath import SE3

from arm.geometry import Sphere, Brick, Capsule
from arm.motion_planning import RRTPlanner, RRTParameter, RRTMap, RRTStarPlanner, InformedRRTStarPlanner


class TestRRT3DPlanner(TestCase):
    def test_rrt(self):
        show_animation = True

        obstacles = [
            # Sphere(SE3.Trans(0.0, 0.0, 0.0), 10.0),
            Brick(SE3.Trans(-8.0, -0.0, 0.0), np.array([4.0, 40.0, 20.0])),
            # Brick(SE3.Trans(10.0, -5.0, 0.0), np.array([20.0, 4.0, 20.0])),
            # Capsule(SE3.Trans(8.0, 11.0, 0.0), 4.0, 20.0),
            # Sphere(SE3.Trans(15.0, 13.0, 12.0), 5.0),

        ]

        rrt_map = RRTMap(area=[(-25.0, 25.0), (-25.0, 20.0), (-25.0, 25.0)], obstacles=obstacles)
        rrt_parameter = RRTParameter(start=[-18.0, -18.0, 0.0], goal=[18.0, 18.0, 0.0], expand_dis=5.0, max_iter=200,
                                     animation=show_animation)
        rrt_planner = RRTPlanner(rrt_map, rrt_parameter)

    def test_rrt_star(self):
        show_animation = True

        obstacles = [
            # Sphere(SE3.Trans(0.0, 0.0, 0.0), 10.0),
            Brick(SE3.Trans(-8.0, -0.0, 0.0), np.array([4.0, 40.0, 20.0])),
            Brick(SE3.Trans(10.0, -5.0, 0.0), np.array([20.0, 4.0, 20.0])),
            # Capsule(SE3.Trans(8.0, 11.0, 0.0), 4.0, 20.0),
            # Sphere(SE3.Trans(15.0, 13.0, 12.0), 5.0),

        ]

        rrt_map = RRTMap(area=[(-25.0, 25.0), (-25.0, 20.0), (-25.0, 25.0)], obstacles=obstacles)
        rrt_parameter = RRTParameter(start=[-18.0, -18.0, 0.0], goal=[18.0, 18.0, 0.0], expand_dis=5.0, max_iter=100,
                                     animation=show_animation)
        rrt_planner = RRTStarPlanner(rrt_map, rrt_parameter)

    def test_informed_rrt_star(self):
        show_animation = True

        obstacles = [
            # Sphere(SE3.Trans(0.0, 0.0, 0.0), 10.0),
            Brick(SE3.Trans(-8.0, -0.0, 0.0), np.array([4.0, 40.0, 20.0])),
            # Brick(SE3.Trans(10.0, -5.0, 0.0), np.array([20.0, 4.0, 20.0])),
            # Capsule(SE3.Trans(8.0, 11.0, 0.0), 4.0, 20.0),
            # Sphere(SE3.Trans(15.0, 13.0, 12.0), 5.0),

        ]

        rrt_map = RRTMap(area=[(-25.0, 25.0), (-25.0, 20.0), (-25.0, 25.0)], obstacles=obstacles)
        rrt_parameter = RRTParameter(start=[-18.0, -18.0, 0.0], goal=[18.0, 18.0, 0.0], expand_dis=5.0, max_iter=100,
                                     animation=show_animation)
        rrt_planner = InformedRRTStarPlanner(rrt_map, rrt_parameter)
