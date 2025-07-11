from unittest import TestCase

import matplotlib.pyplot as plt
import numpy as np
from spatialmath import SE3, SO3
import mujoco
import mujoco.viewer

from arm.robot import IIWA14
from arm.motion_planning import *

robot = IIWA14(tool=np.array([0, 0, 0.2]))


def motion_program(move_fun):
    model = mujoco.MjModel.from_xml_path("../../assets/kuka_iiwa_14/scene.xml")
    data = mujoco.MjData(model)

    q0 = np.array([-0.2, np.pi / 4, 0.2, -np.pi / 4, -0.15, np.pi / 2, 0.0])
    qpos0 = np.zeros_like(model.key_qpos[0, :])
    qpos0[:7] = q0

    robot.set_joint(q0)
    T0 = robot.fkine(q0)
    t0 = T0.t
    R0 = SO3(T0)

    motion_time = 5.0

    time1 = motion_time
    t1 = t0 + np.array([-0.5, -0.2, 0.2])
    R1 = R0
    position_parameter1 = LinePositionParameter(t0, t1)
    attitude_parameter1 = OneAttitudeParameter(R0, R1)
    path_parameter1 = CartesianParameter(position_parameter1, attitude_parameter1)
    velocity_parameter1 = QuinticVelocityParameter(time1)
    trajectory_parameter1 = TrajectoryParameter(path_parameter1, velocity_parameter1)
    trajectory_planner1 = TrajectoryPlanner(trajectory_parameter1)

    time2 = motion_time
    t2 = t1 + np.array([0.5, 0.2, -0.2])
    R2 = R1
    position_parameter2 = LinePositionParameter(t1, t2)
    attitude_parameter2 = OneAttitudeParameter(R1, R2)
    path_parameter2 = CartesianParameter(position_parameter2, attitude_parameter2)
    velocity_parameter2 = QuinticVelocityParameter(time2)
    trajectory_parameter2 = TrajectoryParameter(path_parameter2, velocity_parameter2)
    trajectory_planner2 = TrajectoryPlanner(trajectory_parameter2)

    time3 = motion_time
    t3 = t2 + np.array([-0.8, 0.4, 0.2])
    R3 = R2
    position_parameter3 = LinePositionParameter(t2, t3)
    attitude_parameter3 = OneAttitudeParameter(R2, R3)
    path_parameter3 = CartesianParameter(position_parameter3, attitude_parameter3)
    velocity_parameter3 = QuinticVelocityParameter(time3)
    trajectory_parameter3 = TrajectoryParameter(path_parameter3, velocity_parameter3)
    trajectory_planner3 = TrajectoryPlanner(trajectory_parameter3)

    time4 = motion_time
    t4 = t3 + np.array([0.5, 0.3, -0.2])
    R4 = R3
    position_parameter4 = LinePositionParameter(t3, t4)
    attitude_parameter4 = OneAttitudeParameter(R3, R4)
    path_parameter4 = CartesianParameter(position_parameter4, attitude_parameter4)
    velocity_parameter4 = QuinticVelocityParameter(time4)
    trajectory_parameter4 = TrajectoryParameter(path_parameter4, velocity_parameter4)
    trajectory_planner4 = TrajectoryPlanner(trajectory_parameter4)

    time5 = motion_time
    t5 = t4 + np.array([0.0, -1.2, -0.0])
    R5 = R4
    position_parameter5 = LinePositionParameter(t4, t5)
    attitude_parameter5 = OneAttitudeParameter(R4, R5)
    path_parameter5 = CartesianParameter(position_parameter5, attitude_parameter5)
    velocity_parameter5 = QuinticVelocityParameter(time5)
    trajectory_parameter5 = TrajectoryParameter(path_parameter5, velocity_parameter5)
    trajectory_planner5 = TrajectoryPlanner(trajectory_parameter5)

    time6 = motion_time
    t6 = t5 + np.array([-0.7, -0.0, -0.1])
    R6 = R5
    position_parameter6 = LinePositionParameter(t5, t6)
    attitude_parameter6 = OneAttitudeParameter(R5, R6)
    path_parameter6 = CartesianParameter(position_parameter6, attitude_parameter6)
    velocity_parameter6 = QuinticVelocityParameter(time6)
    trajectory_parameter6 = TrajectoryParameter(path_parameter6, velocity_parameter6)
    trajectory_planner6 = TrajectoryPlanner(trajectory_parameter6)

    time7 = motion_time
    t7 = t6 + np.array([0.5, 0.2, 0.2])
    R7 = R6
    position_parameter7 = LinePositionParameter(t6, t7)
    attitude_parameter7 = OneAttitudeParameter(R6, R7)
    path_parameter7 = CartesianParameter(position_parameter7, attitude_parameter7)
    velocity_parameter7 = QuinticVelocityParameter(time7)
    trajectory_parameter7 = TrajectoryParameter(path_parameter7, velocity_parameter7)
    trajectory_planner7 = TrajectoryPlanner(trajectory_parameter7)

    time8 = motion_time
    t8 = t7 + np.array([0.5, 0.3, -0.1])
    R8 = R7
    position_parameter8 = LinePositionParameter(t7, t8)
    attitude_parameter8 = OneAttitudeParameter(R7, R8)
    path_parameter8 = CartesianParameter(position_parameter8, attitude_parameter8)
    velocity_parameter8 = QuinticVelocityParameter(time8)
    trajectory_parameter8 = TrajectoryParameter(path_parameter8, velocity_parameter8)
    trajectory_planner8 = TrajectoryPlanner(trajectory_parameter8)

    total_time = time1 + time2 + time3 + time4 + time5 + time6 + time7 + time8
    time_step_num = round(total_time / model.opt.timestep) + 1
    times = np.linspace(0, total_time, time_step_num)
    desired_poses = np.zeros((time_step_num, robot.dof))

    phis = np.zeros(time_step_num)
    phi_limits = np.zeros((time_step_num, 2))
    phi_limits_all_low = np.zeros((time_step_num, 7))
    phi_limits_all_up = np.zeros((time_step_num, 7))

    joint_position = np.zeros(7)
    for i, timei in enumerate(times):
        if timei <= time1:
            planner_interpolate = trajectory_planner1.interpolate(timei)
            move_fun(planner_interpolate)
            joint_position = robot.get_joint()
        elif timei <= time1 + time2:
            planner_interpolate = trajectory_planner2.interpolate(timei - time1)
            move_fun(planner_interpolate)
            joint_position = robot.get_joint()
        elif timei <= time1 + time2 + time3:
            planner_interpolate = trajectory_planner3.interpolate(timei - time1 - time2)
            move_fun(planner_interpolate)
            joint_position = robot.get_joint()
        elif timei <= time1 + time2 + time3 + time4:
            planner_interpolate = trajectory_planner4.interpolate(timei - time1 - time2 - time3)
            move_fun(planner_interpolate)
            joint_position = robot.get_joint()
        elif timei <= time1 + time2 + time3 + time4 + time5:
            planner_interpolate = trajectory_planner5.interpolate(timei - time1 - time2 - time3 - time4)
            move_fun(planner_interpolate)
            joint_position = robot.get_joint()
        elif timei <= time1 + time2 + time3 + time4 + time5 + time6:
            planner_interpolate = trajectory_planner6.interpolate(timei - time1 - time2 - time3 - time4 - time5)
            move_fun(planner_interpolate)
            joint_position = robot.get_joint()
        elif timei <= time1 + time2 + time3 + time4 + time5 + time6 + time7:
            planner_interpolate = trajectory_planner7.interpolate(timei - time1 - time2 - time3 - time4 - time5 - time6)
            move_fun(planner_interpolate)
            joint_position = robot.get_joint()
        elif timei <= time1 + time2 + time3 + time4 + time5 + time6 + time7 + time8:
            planner_interpolate = trajectory_planner8.interpolate(timei - time1 - time2 - time3 - time4 - time5 - time6 - time7)
            move_fun(planner_interpolate)
            joint_position = robot.get_joint()
        phis[i] = robot.phi
        phi_limits[i, :] = robot.phi_limit
        phi_limits_all_low[i, :] = robot.phi_limit_all[0, :]
        phi_limits_all_up[i, :] = robot.phi_limit_all[1, :]
        desired_poses[i, :] = joint_position

    time_num = 0
    mujoco.mj_resetData(model, data)
    mujoco.mj_setState(model, data, qpos0, mujoco.mjtState.mjSTATE_QPOS)
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and data.time <= total_time:
            time_num += 1
            if time_num >= time_step_num:
                break

            data.ctrl[:7] = desired_poses[time_num, :]
            data.ctrl[7] = 0

            mujoco.mj_step(model, data)

            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

            viewer.sync()

    plt.figure(1)
    for i in range(robot.dof):
        plt.subplot(4, 2, i + 1)
        plt.plot(times, desired_poses[:, i])
        plt.axhline(robot.q_lim_low[i], c='b')
        plt.axhline(robot.q_lim_up[i], c='b')
    plt.tight_layout()

    plt.figure(2)
    plt.plot(times, phi_limits[:, 0], c='b')
    plt.plot(times, phi_limits[:, 1], c='b')
    plt.plot(times, phis, c='r')
    plt.tight_layout()

    plt.figure(3)
    for i in range(robot.dof):
        plt.subplot(4, 2, i + 1)
        plt.plot(times, phi_limits_all_low[:, i], c='b')
        plt.plot(times, phi_limits_all_up[:, i], c='b')
    plt.tight_layout()

    plt.show()


class TestAvoidance(TestCase):
    def test_move_cartesian(self):
        move_fun = robot.move_cartesian
        motion_program(move_fun)

    def test_move_cartesian_with_avoidance(self):
        move_fun = robot.move_cartesian_with_avoidance
        motion_program(move_fun)
