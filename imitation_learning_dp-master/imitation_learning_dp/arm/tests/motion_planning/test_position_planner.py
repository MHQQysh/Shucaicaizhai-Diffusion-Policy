import numpy as np

# from arm.motion_planning.path_planning.cartesian_planning.position_planning import PositionPlanningModeEnum
# from arm.motion_planning.path_planning.cartesian_planning.position_planning import PositionParameter
from arm.motion_planning.trajectory_planning.path_planning.cartesian_planning import PositionPlanner
from arm.motion_planning.trajectory_planning.path_planning.cartesian_planning import LinePositionParameter, \
    ArcCenterPositionParameter, ArcPointPositionParameter

t0 = -np.array([1.0, 0.0, 0.0]) + 0.2
tc = -np.array([1, 1, 0]) / np.sqrt(2) + 0.2
t1 = -np.array([0, 1.0, 0.0]) + 0.2


def test_position_line_planner():
    # line_attitude_parameter = PositionParameter(p0, p1, 1.0, PositionPlanningModeEnum.LINE)
    line_attitude_parameter = LinePositionParameter(t0, t1)

    planner = PositionPlanner(line_attitude_parameter)
    t = planner.interpolate(0.4)
    print(t)


def test_position_arc_center_planner():
    # arc_center_attitude_parameter = PositionParameter(t0, t1, 1.0, PositionPlanningModeEnum.ARC_CENTER, pc=tc)
    arc_center_attitude_parameter = ArcCenterPositionParameter(t0, t1, tc)
    planner = PositionPlanner(arc_center_attitude_parameter)
    t = planner.interpolate(1.0)
    print(t)


def test_position_arc_point_planner():
    arc_point_position_parameter = ArcPointPositionParameter(t0, t1, tc)
    planner = PositionPlanner(arc_point_position_parameter)
    t = planner.interpolate(1.0)
    print(t)


if __name__ == '__main__':
    # test_position_line_planner()

    # test_position_arc_center_planner()

    test_position_arc_point_planner()
