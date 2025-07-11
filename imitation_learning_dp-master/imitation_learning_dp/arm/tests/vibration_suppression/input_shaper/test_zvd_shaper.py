from unittest import TestCase

import mujoco
import mujoco.viewer
import numpy as np
from spatialmath import SO3, SE3
from matplotlib import pyplot as plt

from arm.controller import ComputedTorqueController
from arm.robot import *
from arm.motion_planning import *
from arm.vibration_suppression import ZVDShaper


class TestZVDShaper(TestCase):
    def test_zvd_shape(self):
        open_shaper = True

        model = mujoco.MjModel.from_xml_path("../../../assets/universal_robots_ur5e/scene.xml")
        data = mujoco.MjData(model)

        robot = UR5e()
        q0 = model.key_qpos[0, :robot.dof]
        robot.set_joint(q0)

        kp = 500.0
        ki = 0.0
        kd = 100.0

        kps = [kp for _ in range(robot.dof)]
        kis = [ki for _ in range(robot.dof)]
        kds = [kd for _ in range(robot.dof)]

        computed_torque_controller = ComputedTorqueController(kps, kis, kds, robot, ts=model.opt.timestep)
        computed_torque_controller.set_qd(q0)

        Td = 0.8673
        omega_d = 2 * np.pi / Td
        zeta = 0.17
        zvd_shaper = ZVDShaper(omega_d, zeta, model.opt.timestep)

        motion_time = 1

        time1 = motion_time
        T0 = robot.get_cartesian()
        t0 = T0.t
        R0 = SO3(T0.R)
        t1 = t0 + np.array([0.0, -0.4, 0.0])
        R1 = SO3(T0.R)
        position_parameter1 = LinePositionParameter(t0, t1)
        attitude_parameter1 = OneAttitudeParameter(R0, R1)
        cartesian_parameter1 = CartesianParameter(position_parameter1, attitude_parameter1)
        velocity_parameter1 = QuinticVelocityParameter(time1)
        trajectory_parameter1 = TrajectoryParameter(cartesian_parameter1, velocity_parameter1)
        trajectory_planner1 = TrajectoryPlanner(trajectory_parameter1)
        times1 = np.arange(0.0, time1, model.opt.timestep)
        trajectory1 = np.zeros((times1.size, 3))
        for i, timei in enumerate(times1):
            planner_interpolate = trajectory_planner1.interpolate(timei)
            trajectory1[i, :] = planner_interpolate.t

        time2 = motion_time
        t2 = t1 + np.array([0.0, 0.8, 0.0])
        R2 = SO3(T0.R)
        position_parameter2 = LinePositionParameter(t1, t2)
        attitude_parameter2 = OneAttitudeParameter(R1, R2)
        cartesian_parameter2 = CartesianParameter(position_parameter2, attitude_parameter2)
        velocity_parameter2 = QuinticVelocityParameter(time2)
        trajectory_parameter2 = TrajectoryParameter(cartesian_parameter2, velocity_parameter2)
        trajectory_planner2 = TrajectoryPlanner(trajectory_parameter2)
        times2 = np.arange(0.0, time2, model.opt.timestep)
        trajectory2 = np.zeros((times2.size, 3))
        for i, timei in enumerate(times2):
            planner_interpolate = trajectory_planner2.interpolate(timei)
            trajectory2[i, :] = planner_interpolate.t

        time3 = motion_time
        t3 = t2 + np.array([-1.0, 0.0, 0.0])
        R3 = SO3(T0.R)
        position_parameter3 = LinePositionParameter(t2, t3)
        attitude_parameter3 = OneAttitudeParameter(R2, R3)
        cartesian_parameter3 = CartesianParameter(position_parameter3, attitude_parameter3)
        velocity_parameter3 = QuinticVelocityParameter(time3)
        trajectory_parameter3 = TrajectoryParameter(cartesian_parameter3, velocity_parameter3)
        trajectory_planner3 = TrajectoryPlanner(trajectory_parameter3)
        times3 = np.arange(0.0, time3, model.opt.timestep)
        trajectory3 = np.zeros((times3.size, 3))
        for i, timei in enumerate(times3):
            planner_interpolate = trajectory_planner3.interpolate(timei)
            trajectory3[i, :] = planner_interpolate.t

        time4 = motion_time
        t4 = t3 + np.array([0.0, -0.8, 0.0])
        R4 = SO3(T0.R)
        position_parameter4 = LinePositionParameter(t3, t4)
        attitude_parameter4 = OneAttitudeParameter(R3, R4)
        cartesian_parameter4 = CartesianParameter(position_parameter4, attitude_parameter4)
        velocity_parameter4 = QuinticVelocityParameter(time4)
        trajectory_parameter4 = TrajectoryParameter(cartesian_parameter4, velocity_parameter4)
        trajectory_planner4 = TrajectoryPlanner(trajectory_parameter4)
        times4 = np.arange(0.0, time4, model.opt.timestep)
        trajectory4 = np.zeros((times4.size, 3))
        for i, timei in enumerate(times4):
            planner_interpolate = trajectory_planner4.interpolate(timei)
            trajectory4[i, :] = planner_interpolate.t

        time5 = motion_time
        t5 = t4 + np.array([1.0, 0.0, 0.0])
        R5 = SO3(T0.R)
        position_parameter5 = LinePositionParameter(t4, t5)
        attitude_parameter5 = OneAttitudeParameter(R4, R5)
        cartesian_parameter5 = CartesianParameter(position_parameter5, attitude_parameter5)
        velocity_parameter5 = QuinticVelocityParameter(time5)
        trajectory_parameter5 = TrajectoryParameter(cartesian_parameter5, velocity_parameter5)
        trajectory_planner5 = TrajectoryPlanner(trajectory_parameter5)
        times5 = np.arange(0.0, time5, model.opt.timestep)
        trajectory5 = np.zeros((times5.size, 3))
        for i, timei in enumerate(times5):
            planner_interpolate = trajectory_planner5.interpolate(timei)
            trajectory5[i, :] = planner_interpolate.t

        time6 = motion_time
        t6 = t5 + np.array([0.0, 0.4, 0.0])
        R6 = SO3(T0.R)
        position_parameter6 = LinePositionParameter(t5, t6)
        attitude_parameter6 = OneAttitudeParameter(R5, R6)
        cartesian_parameter6 = CartesianParameter(position_parameter6, attitude_parameter6)
        velocity_parameter6 = QuinticVelocityParameter(time6)
        trajectory_parameter6 = TrajectoryParameter(cartesian_parameter6, velocity_parameter6)
        trajectory_planner6 = TrajectoryPlanner(trajectory_parameter6)
        times6 = np.arange(0.0, time6, model.opt.timestep)
        trajectory6 = np.zeros((times6.size, 3))
        for i, timei in enumerate(times6):
            planner_interpolate = trajectory_planner6.interpolate(timei)
            trajectory6[i, :] = planner_interpolate.t

        if open_shaper:
            trajectory1 = zvd_shaper.shape(trajectory1)
            trajectory2 = zvd_shaper.shape(trajectory2)
            trajectory3 = zvd_shaper.shape(trajectory3)
            trajectory4 = zvd_shaper.shape(trajectory4)
            trajectory5 = zvd_shaper.shape(trajectory5)
            trajectory6 = zvd_shaper.shape(trajectory6)

        total_time = 30
        time_step_num = round(total_time / model.opt.timestep) + 1
        times = np.linspace(0, total_time, time_step_num)
        desired_poses = np.zeros((time_step_num, robot.dof))
        actual_poses = np.zeros_like(desired_poses)
        tool_poses = np.zeros((time_step_num, 3))

        cartesian_poses = np.zeros((time_step_num, 3))
        interval1 = 1500
        interval2 = 1500
        interval3 = 1500
        interval4 = 1500
        interval5 = 1500
        interval6 = 1500
        for i in range(time_step_num):
            if i < interval1:
                cartesian_poses[i, :] = trajectory1[0, :]
            elif i < interval1 + trajectory1.shape[0]:
                cartesian_poses[i, :] = trajectory1[i - interval1, :]
            elif i < interval1 + trajectory1.shape[0] + interval2:
                cartesian_poses[i, :] = trajectory2[0, :]
            elif i < interval1 + trajectory1.shape[0] + interval2 + trajectory2.shape[0]:
                cartesian_poses[i, :] = trajectory2[i - (interval1 + trajectory1.shape[0] + interval2), :]
            elif i < interval1 + trajectory1.shape[0] + interval2 + trajectory2.shape[0] + interval3:
                cartesian_poses[i, :] = trajectory3[0, :]
            elif i < interval1 + trajectory1.shape[0] + interval2 + trajectory2.shape[0] + interval3 + \
                    trajectory3.shape[0]:
                cartesian_poses[i, :] = trajectory3[i - (
                        interval1 + trajectory1.shape[0] + interval2 + trajectory2.shape[0] + interval3), :]
            elif i < interval1 + trajectory1.shape[0] + interval2 + trajectory2.shape[0] + interval3 + \
                    trajectory3.shape[0] + interval4:
                cartesian_poses[i, :] = trajectory4[0, :]
            elif i < interval1 + trajectory1.shape[0] + interval2 + trajectory2.shape[0] + interval3 + \
                    trajectory3.shape[0] + interval4 + trajectory4.shape[0]:
                cartesian_poses[i, :] = trajectory4[i - (
                        interval1 + trajectory1.shape[0] + interval2 + trajectory2.shape[0] + interval3 +
                        trajectory3.shape[0] + interval4 + trajectory4.shape[0]), :]
            elif i < interval1 + trajectory1.shape[0] + interval2 + trajectory2.shape[0] + interval3 + \
                    trajectory3.shape[0] + interval4 + trajectory4.shape[0] + interval5:
                cartesian_poses[i, :] = trajectory5[0, :]
            elif i < interval1 + trajectory1.shape[0] + interval2 + trajectory2.shape[0] + interval3 + \
                    trajectory3.shape[0] + interval4 + trajectory4.shape[0] + interval5 + trajectory5.shape[0]:
                cartesian_poses[i, :] = trajectory5[i - (
                        interval1 + trajectory1.shape[0] + interval2 + trajectory2.shape[0] + interval3 +
                        trajectory3.shape[0] + interval4 + trajectory4.shape[0] + interval5 + trajectory5.shape[0]), :]
            elif i < interval1 + trajectory1.shape[0] + interval2 + trajectory2.shape[0] + interval3 + \
                    trajectory3.shape[0] + interval4 + trajectory4.shape[0] + interval5 + trajectory5.shape[
                0] + interval6:
                cartesian_poses[i, :] = trajectory6[0, :]
            elif i < interval1 + trajectory1.shape[0] + interval2 + trajectory2.shape[0] + interval3 + \
                    trajectory3.shape[0] + interval4 + trajectory4.shape[0] + interval5 + trajectory5.shape[
                0] + interval6 + trajectory6.shape[0]:
                cartesian_poses[i, :] = trajectory6[i - (
                        interval1 + trajectory1.shape[0] + interval2 + trajectory2.shape[0] + interval3 +
                        trajectory3.shape[0] + interval4 + trajectory4.shape[0] + interval5 + trajectory5.shape[
                            0] + interval6 + trajectory6.shape[0]), :]
            else:
                cartesian_poses[i, :] = cartesian_poses[i - 1, :]

        for i, timei in enumerate(times):
            cartesian_pos = cartesian_poses[i, :]
            R = R0
            T = SE3(cartesian_pos) * SE3(R)
            robot.move_cartesian(T)
            joint_position = robot.get_joint()

            desired_poses[i, :] = joint_position

        time_num = 0
        mujoco.mj_setState(model, data, np.hstack((model.key_qpos[0, :], model.key_qvel[0, :])),
                           mujoco.mjtState.mjSTATE_PHYSICS)
        mujoco.mj_forward(model, data)

        actual_poses[time_num, :] = data.sensordata
        tool_poses[time_num, :] = data.site('tool_site').xpos

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time <= total_time:
                time_num += 1

                if time_num >= time_step_num:
                    break

                desired_pos = desired_poses[time_num, :]
                sensor_data = data.sensordata
                actual_pos = sensor_data.copy()

                data.ctrl[:] = computed_torque_controller.control(desired_pos, sensor_data)

                actual_poses[time_num, :] = actual_pos
                tool_poses[time_num, :] = data.site('tool_site').xpos

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
        for i in range(3):
            plt.subplot(3, 1, i + 1)
            plt.plot(times, tool_poses[:, i], '-', label='tool position')
            plt.legend()
        plt.tight_layout()

        plt.show()
