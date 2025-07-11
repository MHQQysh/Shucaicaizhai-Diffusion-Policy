from unittest import TestCase

import mujoco
import mujoco.viewer
import numpy as np

from arm.controller import AdaptiveController
from matplotlib import pyplot as plt

from arm.robot import *
from arm.motion_planning import *
from spatialmath import SO3


class TestAdaptiveController(TestCase):
    def test_adaptive_control1(self):
        model = mujoco.MjModel.from_xml_path("../../assets/universal_robots_ur5e/scene.xml")
        data = mujoco.MjData(model)

        robot = UR5e()
        q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        robot.set_joint(q0)

        kds = [20, 20, 20, 5, 1, 0.1]
        controller = AdaptiveController(kds, robot, ts=model.opt.timestep)

        motion_time = 1

        time1 = motion_time
        q1 = np.array([0.0, 0.0, np.pi / 2, 0.0, -np.pi / 2, 0.0])
        joint_parameter1 = JointParameter(q0, q1)
        velocity_parameter1 = QuinticVelocityParameter(time1)
        trajectory_parameter1 = TrajectoryParameter(joint_parameter1, velocity_parameter1)
        trajectory_planner1 = TrajectoryPlanner(trajectory_parameter1)

        time2 = motion_time
        robot.set_joint(q1)
        T1 = robot.get_cartesian()
        t1 = T1.t
        R1 = SO3(T1.R)
        t2 = t1 + np.array([0.0, -0.3, 0.0])
        R2 = SO3(T1.R)
        position_parameter2 = LinePositionParameter(t1, t2)
        attitude_parameter2 = OneAttitudeParameter(R1, R2)
        cartesian_parameter2 = CartesianParameter(position_parameter2, attitude_parameter2)
        velocity_parameter2 = QuinticVelocityParameter(time2)
        trajectory_parameter2 = TrajectoryParameter(cartesian_parameter2, velocity_parameter2)
        trajectory_planner2 = TrajectoryPlanner(trajectory_parameter2)

        time3 = motion_time
        t3 = t2 + np.array([0.3, 0.3, -0.3])
        R3 = R2
        position_parameter3 = LinePositionParameter(t2, t3)
        attitude_parameter3 = OneAttitudeParameter(R2, R3)
        cartesian_parameter3 = CartesianParameter(position_parameter3, attitude_parameter3)
        velocity_parameter3 = QuinticVelocityParameter(time3)
        trajectory_parameter3 = TrajectoryParameter(cartesian_parameter3, velocity_parameter3)
        trajectory_planner3 = TrajectoryPlanner(trajectory_parameter3)

        time4 = motion_time
        t4 = t3 + np.array([-0.3, 0.3, 0.3])
        R4 = R3
        position_parameter4 = LinePositionParameter(t3, t4)
        attitude_parameter4 = OneAttitudeParameter(R3, R4)
        cartesian_parameter4 = CartesianParameter(position_parameter4, attitude_parameter4)
        velocity_parameter4 = QuinticVelocityParameter(time4)
        trajectory_parameter4 = TrajectoryParameter(cartesian_parameter4, velocity_parameter4)
        trajectory_planner4 = TrajectoryPlanner(trajectory_parameter4)

        time5 = motion_time
        t5 = t4 + np.array([-0.6, 0.0, 0.0])
        R5 = R4
        position_parameter5 = LinePositionParameter(t4, t5)
        attitude_parameter5 = OneAttitudeParameter(R4, R5)
        cartesian_parameter5 = CartesianParameter(position_parameter5, attitude_parameter5)
        velocity_parameter5 = QuinticVelocityParameter(time5)
        trajectory_parameter5 = TrajectoryParameter(cartesian_parameter5, velocity_parameter5)
        trajectory_planner5 = TrajectoryPlanner(trajectory_parameter5)

        time6 = motion_time
        t6 = t5 + np.array([0.0, -0.8, 0.0])
        R6 = R5
        tc6 = (t5 + t6) / 2.0
        tc6[0] -= 0.2
        tc6[2] -= 0.3
        position_parameter6 = ArcPointPositionParameter(t5, t6, tc6)
        attitude_parameter6 = OneAttitudeParameter(R5, R6)
        cartesian_parameter6 = CartesianParameter(position_parameter6, attitude_parameter6)
        velocity_parameter6 = QuinticVelocityParameter(time6)
        trajectory_parameter6 = TrajectoryParameter(cartesian_parameter6, velocity_parameter6)
        trajectory_planner6 = TrajectoryPlanner(trajectory_parameter6)

        time7 = motion_time
        t7 = t1
        R7 = R1
        position_parameter7 = LinePositionParameter(t6, t7)
        attitude_parameter7 = OneAttitudeParameter(R6, R7)
        cartesian_parameter7 = CartesianParameter(position_parameter7, attitude_parameter7)
        velocity_parameter7 = QuinticVelocityParameter(time7)
        trajectory_parameter7 = TrajectoryParameter(cartesian_parameter7, velocity_parameter7)
        trajectory_planner7 = TrajectoryPlanner(trajectory_parameter7)

        time8 = motion_time

        total_time = time1 + time2 + time3 + time4 + time5 + time6 + time7 + time8
        time_step_num = round(total_time / model.opt.timestep) + 1
        desired_poses = np.zeros((time_step_num, robot.dof))
        actual_poses = np.zeros_like(desired_poses)
        error_poses = np.zeros_like(desired_poses)
        times = np.linspace(0, total_time, time_step_num)

        joint_position = np.zeros(6)
        for i, timei in enumerate(times):
            if timei < time1:
                planner_interpolate = trajectory_planner1.interpolate(timei)
                joint_position = planner_interpolate
                robot.move_joint(joint_position)

            elif timei <= time1 + time2:
                planner_interpolate = trajectory_planner2.interpolate(timei - time1)
                robot.move_cartesian(planner_interpolate)
                joint_position = robot.get_joint()

            elif timei <= time1 + time2 + time3:
                planner_interpolate = trajectory_planner3.interpolate(timei - time1 - time2)
                robot.move_cartesian(planner_interpolate)
                joint_position = robot.get_joint()

            elif timei <= time1 + time2 + time3 + time4:
                planner_interpolate = trajectory_planner4.interpolate(timei - time1 - time2 - time3)
                robot.move_cartesian(planner_interpolate)
                joint_position = robot.get_joint()

            elif timei <= time1 + time2 + time3 + time4 + time5:
                planner_interpolate = trajectory_planner5.interpolate(timei - time1 - time2 - time3 - time4)
                robot.move_cartesian(planner_interpolate)
                joint_position = robot.get_joint()

            elif timei <= time1 + time2 + time3 + time4 + time5 + time6:
                planner_interpolate = trajectory_planner6.interpolate(timei - time1 - time2 - time3 - time4 - time5)
                robot.move_cartesian(planner_interpolate)
                joint_position = robot.get_joint()

            elif timei <= time1 + time2 + time3 + time4 + time5 + time6 + time7:
                planner_interpolate = trajectory_planner7.interpolate(
                    timei - time1 - time2 - time3 - time4 - time5 - time6)
                robot.move_cartesian(planner_interpolate)
                joint_position = robot.get_joint()

            desired_poses[i, :] = joint_position

        time_num = 0
        mujoco.mj_resetData(model, data)
        initial_state = np.zeros(mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_PHYSICS))
        mujoco.mj_setState(model, data, initial_state, mujoco.mjtState.mjSTATE_PHYSICS)
        mujoco.mj_forward(model, data)

        actual_poses[time_num, :] = data.qpos[:6].copy()

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time <= total_time:
                time_num += 1

                if time_num >= time_step_num:
                    break

                desired_pos = desired_poses[time_num, :].copy()
                actual_pos = data.qpos[:6].copy()
                error_pos = desired_pos - actual_pos

                actual_poses[time_num, :] = actual_pos.copy()
                error_poses[time_num, :] = error_pos.copy()
                torques = controller.control(desired_pos, data.qpos[:6])

                mujoco.mj_setState(model, data, torques, mujoco.mjtState.mjSTATE_CTRL)
                mujoco.mj_step(model, data)

                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

                viewer.sync()

        plt.figure(1)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, desired_poses[:, i], '-', label='desired position')
            plt.plot(times, actual_poses[:, i], '--', label='actual position')
            plt.legend()
        plt.tight_layout()

        plt.figure(2)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, error_poses[:, i], '-', label='position error')
            plt.legend()
        plt.tight_layout()

        plt.show()

    def test_adaptive_control2(self):
        model = mujoco.MjModel.from_xml_path("../../assets/universal_robots_ur5e/scene.xml")
        data = mujoco.MjData(model)

        robot = UR5e()
        q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        robot.set_joint(q0)

        kds = [20, 20, 20, 5, 1, 0.1]
        controller = AdaptiveController(kds, robot, ts=model.opt.timestep)

        motion_time = 1

        time1 = motion_time
        q1 = np.array([0.0, 0.0, np.pi / 2, 0.0, -np.pi / 2, 0.0])
        joint_parameter1 = JointParameter(q0, q1)
        velocity_parameter1 = QuinticVelocityParameter(time1)
        trajectory_parameter1 = TrajectoryParameter(joint_parameter1, velocity_parameter1)
        trajectory_planner1 = TrajectoryPlanner(trajectory_parameter1)

        time2 = motion_time
        robot.set_joint(q1)
        T1 = robot.get_cartesian()
        t1 = T1.t
        R1 = SO3(T1.R)
        t2 = t1 + np.array([0.0, -0.3, 0.0])
        R2 = SO3(T1.R)
        position_parameter2 = LinePositionParameter(t1, t2)
        attitude_parameter2 = OneAttitudeParameter(R1, R2)
        cartesian_parameter2 = CartesianParameter(position_parameter2, attitude_parameter2)
        velocity_parameter2 = QuinticVelocityParameter(time2)
        trajectory_parameter2 = TrajectoryParameter(cartesian_parameter2, velocity_parameter2)
        trajectory_planner2 = TrajectoryPlanner(trajectory_parameter2)

        time3 = motion_time
        t3 = t2 + np.array([0.3, 0.3, -0.3])
        R3 = R2
        position_parameter3 = LinePositionParameter(t2, t3)
        attitude_parameter3 = OneAttitudeParameter(R2, R3)
        cartesian_parameter3 = CartesianParameter(position_parameter3, attitude_parameter3)
        velocity_parameter3 = QuinticVelocityParameter(time3)
        trajectory_parameter3 = TrajectoryParameter(cartesian_parameter3, velocity_parameter3)
        trajectory_planner3 = TrajectoryPlanner(trajectory_parameter3)

        time4 = motion_time
        t4 = t3 + np.array([-0.3, 0.3, 0.3])
        R4 = R3
        position_parameter4 = LinePositionParameter(t3, t4)
        attitude_parameter4 = OneAttitudeParameter(R3, R4)
        cartesian_parameter4 = CartesianParameter(position_parameter4, attitude_parameter4)
        velocity_parameter4 = QuinticVelocityParameter(time4)
        trajectory_parameter4 = TrajectoryParameter(cartesian_parameter4, velocity_parameter4)
        trajectory_planner4 = TrajectoryPlanner(trajectory_parameter4)

        time5 = motion_time
        t5 = t4 + np.array([-0.6, 0.0, 0.0])
        R5 = R4
        position_parameter5 = LinePositionParameter(t4, t5)
        attitude_parameter5 = OneAttitudeParameter(R4, R5)
        cartesian_parameter5 = CartesianParameter(position_parameter5, attitude_parameter5)
        velocity_parameter5 = QuinticVelocityParameter(time5)
        trajectory_parameter5 = TrajectoryParameter(cartesian_parameter5, velocity_parameter5)
        trajectory_planner5 = TrajectoryPlanner(trajectory_parameter5)

        time6 = motion_time
        t6 = t5 + np.array([0.0, -0.8, 0.0])
        R6 = R5
        tc6 = (t5 + t6) / 2.0
        tc6[0] -= 0.2
        tc6[2] -= 0.3
        position_parameter6 = ArcPointPositionParameter(t5, t6, tc6)
        attitude_parameter6 = OneAttitudeParameter(R5, R6)
        cartesian_parameter6 = CartesianParameter(position_parameter6, attitude_parameter6)
        velocity_parameter6 = QuinticVelocityParameter(time6)
        trajectory_parameter6 = TrajectoryParameter(cartesian_parameter6, velocity_parameter6)
        trajectory_planner6 = TrajectoryPlanner(trajectory_parameter6)

        time7 = motion_time
        t7 = t1
        R7 = R1
        position_parameter7 = LinePositionParameter(t6, t7)
        attitude_parameter7 = OneAttitudeParameter(R6, R7)
        cartesian_parameter7 = CartesianParameter(position_parameter7, attitude_parameter7)
        velocity_parameter7 = QuinticVelocityParameter(time7)
        trajectory_parameter7 = TrajectoryParameter(cartesian_parameter7, velocity_parameter7)
        trajectory_planner7 = TrajectoryPlanner(trajectory_parameter7)

        time8 = motion_time
        q7 = np.array([2.0 * np.pi, 0.0, np.pi / 2.0, 0.0, -np.pi / 2.0, 2.0 * np.pi])
        q8 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        joint_parameter8 = JointParameter(q7, q8)
        velocity_parameter8 = QuinticVelocityParameter(time8)
        trajectory_parameter8 = TrajectoryParameter(joint_parameter8, velocity_parameter8)
        trajectory_planner8 = TrajectoryPlanner(trajectory_parameter8)

        total_time = (time1 + time2 + time3 + time4 + time5 + time6 + time7 + time8) * 3
        time_step_num = round(total_time / model.opt.timestep) + 1
        desired_poses = np.zeros((time_step_num, robot.dof))
        actual_poses = np.zeros_like(desired_poses)
        error_poses = np.zeros_like(desired_poses)
        times = np.linspace(0, total_time, time_step_num)

        joint_position = np.zeros(6)
        for i, timei in enumerate(times):
            if timei < time1:
                planner_interpolate = trajectory_planner1.interpolate(timei)
                joint_position = planner_interpolate
                robot.move_joint(joint_position)

            elif timei <= time1 + time2:
                planner_interpolate = trajectory_planner2.interpolate(timei - time1)
                robot.move_cartesian(planner_interpolate)
                joint_position = robot.get_joint()

            elif timei <= time1 + time2 + time3:
                planner_interpolate = trajectory_planner3.interpolate(timei - time1 - time2)
                robot.move_cartesian(planner_interpolate)
                joint_position = robot.get_joint()

            elif timei <= time1 + time2 + time3 + time4:
                planner_interpolate = trajectory_planner4.interpolate(timei - time1 - time2 - time3)
                robot.move_cartesian(planner_interpolate)
                joint_position = robot.get_joint()

            elif timei <= time1 + time2 + time3 + time4 + time5:
                planner_interpolate = trajectory_planner5.interpolate(timei - time1 - time2 - time3 - time4)
                robot.move_cartesian(planner_interpolate)
                joint_position = robot.get_joint()

            elif timei <= time1 + time2 + time3 + time4 + time5 + time6:
                planner_interpolate = trajectory_planner6.interpolate(timei - time1 - time2 - time3 - time4 - time5)
                robot.move_cartesian(planner_interpolate)
                joint_position = robot.get_joint()

            elif timei <= time1 + time2 + time3 + time4 + time5 + time6 + time7:
                planner_interpolate = trajectory_planner7.interpolate(
                    timei - time1 - time2 - time3 - time4 - time5 - time6)
                robot.move_cartesian(planner_interpolate)
                joint_position = robot.get_joint()

            elif timei <= time1 + time2 + time3 + time4 + time5 + time6 + time7 + time8:
                planner_interpolate = trajectory_planner8.interpolate(
                    timei - time1 - time2 - time3 - time4 - time5 - time6 - time7)
                joint_position = planner_interpolate
                robot.move_joint(joint_position)

            desired_poses[i, :] = joint_position

        time_num = 0
        mujoco.mj_resetData(model, data)
        initial_state = np.zeros(mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_PHYSICS))
        mujoco.mj_setState(model, data, initial_state, mujoco.mjtState.mjSTATE_PHYSICS)
        mujoco.mj_forward(model, data)

        desired_poses[4001:8001] = desired_poses[1:4001]
        desired_poses[8001:] = desired_poses[1:4001]
        actual_poses[time_num, :] = data.qpos[:6].copy()

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time <= total_time:
                time_num += 1

                if time_num >= time_step_num:
                    break

                desired_pos = desired_poses[time_num, :].copy()
                actual_pos = data.qpos[:6].copy()
                error_pos = desired_pos - actual_pos

                actual_poses[time_num, :] = actual_pos.copy()
                error_poses[time_num, :] = error_pos.copy()
                torques = controller.control(desired_pos, data.qpos[:6])

                mujoco.mj_setState(model, data, torques, mujoco.mjtState.mjSTATE_CTRL)
                mujoco.mj_step(model, data)

                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

                viewer.sync()

        plt.figure(1)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, desired_poses[:, i], '-', label='desired position')
            plt.plot(times, actual_poses[:, i], '--', label='actual position')
            plt.legend()
        plt.tight_layout()

        plt.figure(2)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, error_poses[:, i], '-', label='position error')
            plt.legend()
        plt.tight_layout()

        plt.show()
