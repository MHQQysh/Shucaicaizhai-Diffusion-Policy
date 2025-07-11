from unittest import TestCase

import matplotlib.pyplot as plt
import numpy as np

import mujoco
import mujoco.viewer

from arm.robot import UR5e
from arm.controller import ComputedTorqueController
from arm.motion_planning import *


class TestTimeOptimalPlanner(TestCase):

    def test_time_optimal_planner(self):
        model = mujoco.MjModel.from_xml_path("../../assets/universal_robots_ur5e/scene.xml")
        data = mujoco.MjData(model)

        robot = UR5e()
        q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        robot.set_joint(q0)

        phi = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0])
        alpha = np.array([10.0, 10.0, 10.0, 10.0, 10.0, 10.0])
        mu = np.array([100.0, 100.0, 100.0, 100.0, 100.0, 100.0])

        kp = 100.0
        ki = 0.0
        kd = 20.0

        kps = [kp for _ in range(robot.dof)]
        kis = [ki for _ in range(robot.dof)]
        kds = [kd for _ in range(robot.dof)]

        computed_torque_controller = ComputedTorqueController(kps, kis, kds, robot, ts=model.opt.timestep)

        num = 101
        s = np.linspace(0, 1, num)

        q1 = np.array([0.0, 0.0, np.pi / 2, 0.0, -np.pi / 2, 0.0])
        joint_parameter1 = JointParameter(q0, q1)
        joint_planner1 = JointPlanner(joint_parameter1)
        qs = np.zeros((s.size, robot.dof))
        dqs = np.zeros_like(qs)
        ddqs = np.zeros_like(qs)
        for i in range(s.size):
            qs[i, :] = joint_planner1.interpolate(s[i])
            dqs[i, :] = joint_planner1.interpolate_derivative(s[i])
            ddqs[i, :] = joint_planner1.interpolate_second_derivative(s[i])
        time_optimal_planner1 = TimeOptimalPlanner(qs, dqs, ddqs, robot, phi, alpha, mu, model.opt.timestep)

        q2 = q1 + np.array([-np.pi / 4, np.pi / 4, 0.0, -np.pi / 4, 0.0, 0.0])
        joint_parameter2 = JointParameter(q1, q2)
        joint_planner2 = JointPlanner(joint_parameter2)
        qs = np.zeros((s.size, robot.dof))
        dqs = np.zeros_like(qs)
        ddqs = np.zeros_like(qs)
        for i in range(s.size):
            qs[i, :] = joint_planner2.interpolate(s[i])
            dqs[i, :] = joint_planner2.interpolate_derivative(s[i])
            ddqs[i, :] = joint_planner2.interpolate_second_derivative(s[i])
        time_optimal_planner2 = TimeOptimalPlanner(qs, dqs, ddqs, robot, phi, alpha, mu, model.opt.timestep)

        q3 = q2 + np.array([np.pi / 2, np.pi / 4, -np.pi / 2, np.pi / 4, 0.0, np.pi])
        joint_parameter3 = JointParameter(q2, q3)
        joint_planner3 = JointPlanner(joint_parameter3)
        qs = np.zeros((s.size, robot.dof))
        dqs = np.zeros_like(qs)
        ddqs = np.zeros_like(qs)
        for i in range(s.size):
            qs[i, :] = joint_planner3.interpolate(s[i])
            dqs[i, :] = joint_planner3.interpolate_derivative(s[i])
            ddqs[i, :] = joint_planner3.interpolate_second_derivative(s[i])
        time_optimal_planner3 = TimeOptimalPlanner(qs, dqs, ddqs, robot, phi, alpha, mu, model.opt.timestep)

        q4 = q3 + np.array([np.pi / 4, -np.pi / 2, np.pi / 2, 0.0, 0.0, 0.0])
        joint_parameter4 = JointParameter(q3, q4)
        joint_planner4 = JointPlanner(joint_parameter4)
        qs = np.zeros((s.size, robot.dof))
        dqs = np.zeros_like(qs)
        ddqs = np.zeros_like(qs)
        for i in range(s.size):
            qs[i, :] = joint_planner4.interpolate(s[i])
            dqs[i, :] = joint_planner4.interpolate_derivative(s[i])
            ddqs[i, :] = joint_planner4.interpolate_second_derivative(s[i])
        time_optimal_planner4 = TimeOptimalPlanner(qs, dqs, ddqs, robot, phi, alpha, mu, model.opt.timestep)

        q5 = q4 + np.array([np.pi / 2, np.pi / 4, -np.pi / 4, 0.0, 0.0, -np.pi / 2])
        joint_parameter5 = JointParameter(q4, q5)
        joint_planner5 = JointPlanner(joint_parameter5)
        qs = np.zeros((s.size, robot.dof))
        dqs = np.zeros_like(qs)
        ddqs = np.zeros_like(qs)
        for i in range(s.size):
            qs[i, :] = joint_planner5.interpolate(s[i])
            dqs[i, :] = joint_planner5.interpolate_derivative(s[i])
            ddqs[i, :] = joint_planner5.interpolate_second_derivative(s[i])
        time_optimal_planner5 = TimeOptimalPlanner(qs, dqs, ddqs, robot, phi, alpha, mu, model.opt.timestep)

        q6 = q5 + np.array([np.pi / 2, -np.pi / 6, np.pi / 6, 0.0, 0.0, -np.pi / 2])
        joint_parameter6 = JointParameter(q5, q6)
        joint_planner6 = JointPlanner(joint_parameter6)
        qs = np.zeros((s.size, robot.dof))
        dqs = np.zeros_like(qs)
        ddqs = np.zeros_like(qs)
        for i in range(s.size):
            qs[i, :] = joint_planner6.interpolate(s[i])
            dqs[i, :] = joint_planner6.interpolate_derivative(s[i])
            ddqs[i, :] = joint_planner6.interpolate_second_derivative(s[i])
        time_optimal_planner6 = TimeOptimalPlanner(qs, dqs, ddqs, robot, phi, alpha, mu, model.opt.timestep)

        q7 = q6 + np.array([np.pi / 2, np.pi / 4, -np.pi / 8, -np.pi / 8, 0.0, 0.0])
        joint_parameter7 = JointParameter(q6, q7)
        joint_planner7 = JointPlanner(joint_parameter7)
        qs = np.zeros((s.size, robot.dof))
        dqs = np.zeros_like(qs)
        ddqs = np.zeros_like(qs)
        for i in range(s.size):
            qs[i, :] = joint_planner7.interpolate(s[i])
            dqs[i, :] = joint_planner7.interpolate_derivative(s[i])
            ddqs[i, :] = joint_planner7.interpolate_second_derivative(s[i])
        time_optimal_planner7 = TimeOptimalPlanner(qs, dqs, ddqs, robot, phi, alpha, mu, model.opt.timestep)

        q8 = np.array([np.pi * 2, 0.0, 0.0, 0.0, 0.0, 0.0])
        joint_parameter8 = JointParameter(q7, q8)
        joint_planner8 = JointPlanner(joint_parameter8)
        qs = np.zeros((s.size, robot.dof))
        dqs = np.zeros_like(qs)
        ddqs = np.zeros_like(qs)
        for i in range(s.size):
            qs[i, :] = joint_planner8.interpolate(s[i])
            dqs[i, :] = joint_planner8.interpolate_derivative(s[i])
            ddqs[i, :] = joint_planner8.interpolate_second_derivative(s[i])
        time_optimal_planner8 = TimeOptimalPlanner(qs, dqs, ddqs, robot, phi, alpha, mu, model.opt.timestep)

        total_time = time_optimal_planner1.tf + time_optimal_planner2.tf + time_optimal_planner3.tf \
                     + time_optimal_planner4.tf + time_optimal_planner5.tf + time_optimal_planner6.tf \
                     + time_optimal_planner7.tf + time_optimal_planner8.tf
        time_step_num = round(total_time / model.opt.timestep) + 1
        desired_poses = np.zeros((time_step_num, robot.dof))
        desired_vels = np.zeros_like(desired_poses)
        desired_accs = np.zeros_like(desired_poses)
        actual_poses = np.zeros_like(desired_poses)
        actual_vels = np.zeros_like(desired_poses)
        actual_accs = np.zeros_like(desired_poses)
        motor_ctrls = np.zeros_like(desired_poses)
        times = np.linspace(0, total_time, time_step_num)

        joint_position = np.zeros(6)
        for i, timei in enumerate(times):
            if timei < time_optimal_planner1.tf:
                joint_position = time_optimal_planner1.interpolate(timei)
            elif timei < time_optimal_planner1.tf + time_optimal_planner2.tf:
                joint_position = time_optimal_planner2.interpolate(timei - time_optimal_planner1.tf)
            elif timei < time_optimal_planner1.tf + time_optimal_planner2.tf + time_optimal_planner3.tf:
                joint_position = time_optimal_planner3.interpolate(
                    timei - time_optimal_planner1.tf - time_optimal_planner2.tf)
            elif timei < time_optimal_planner1.tf + time_optimal_planner2.tf + time_optimal_planner3.tf \
                    + time_optimal_planner4.tf:
                joint_position = time_optimal_planner4.interpolate(
                    timei - time_optimal_planner1.tf - time_optimal_planner2.tf - time_optimal_planner3.tf)
            elif timei < time_optimal_planner1.tf + time_optimal_planner2.tf + time_optimal_planner3.tf \
                    + time_optimal_planner4.tf + time_optimal_planner5.tf:
                joint_position = time_optimal_planner5.interpolate(
                    timei - time_optimal_planner1.tf - time_optimal_planner2.tf - time_optimal_planner3.tf
                    - time_optimal_planner4.tf)
            elif timei < time_optimal_planner1.tf + time_optimal_planner2.tf + time_optimal_planner3.tf \
                    + time_optimal_planner4.tf + time_optimal_planner5.tf + time_optimal_planner6.tf:
                joint_position = time_optimal_planner6.interpolate(
                    timei - time_optimal_planner1.tf - time_optimal_planner2.tf - time_optimal_planner3.tf
                    - time_optimal_planner4.tf - time_optimal_planner5.tf)
            elif timei < time_optimal_planner1.tf + time_optimal_planner2.tf + time_optimal_planner3.tf \
                    + time_optimal_planner4.tf + time_optimal_planner5.tf + time_optimal_planner6.tf \
                    + time_optimal_planner7.tf:
                joint_position = time_optimal_planner7.interpolate(
                    timei - time_optimal_planner1.tf - time_optimal_planner2.tf - time_optimal_planner3.tf
                    - time_optimal_planner4.tf - time_optimal_planner5.tf - time_optimal_planner6.tf)
            elif timei < time_optimal_planner1.tf + time_optimal_planner2.tf + time_optimal_planner3.tf \
                    + time_optimal_planner4.tf + time_optimal_planner5.tf + time_optimal_planner6.tf \
                    + time_optimal_planner7.tf + time_optimal_planner8.tf:
                joint_position = time_optimal_planner8.interpolate(
                    timei - time_optimal_planner1.tf - time_optimal_planner2.tf - time_optimal_planner3.tf
                    - time_optimal_planner4.tf - time_optimal_planner5.tf - time_optimal_planner6.tf
                    - time_optimal_planner7.tf)
            desired_poses[i, :] = joint_position
            if i > 0:
                desired_vels[i, :] = (desired_poses[i, :] - desired_poses[i - 1, :]) / model.opt.timestep
                desired_accs[i, :] = (desired_vels[i, :] - desired_vels[i - 1, :]) / model.opt.timestep

        time_num = 0
        mujoco.mj_resetData(model, data)
        initial_state = np.zeros(mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_PHYSICS))
        mujoco.mj_setState(model, data, initial_state, mujoco.mjtState.mjSTATE_PHYSICS)
        mujoco.mj_forward(model, data)

        sensor_data = data.sensordata.copy()
        actual_pos = sensor_data.copy()
        actual_pos_prev = actual_pos.copy()
        actual_vel = (actual_pos - actual_pos_prev) / model.opt.timestep
        actual_vel_prev = actual_vel.copy()
        actual_acc = (actual_vel - actual_vel_prev) / model.opt.timestep

        actual_poses[time_num, :] = sensor_data.copy()
        actual_vels[time_num, :] = actual_vel.copy()
        actual_accs[time_num, :] = actual_acc.copy()
        motor_ctrls[time_num, :] = data.ctrl.copy()

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time <= total_time:
                time_num += 1

                if time_num >= time_step_num:
                    break

                mujoco.mj_step(model, data)

                desired_pos = desired_poses[time_num, :].copy()
                sensor_data = data.sensordata.copy()
                actual_pos_prev = actual_pos.copy()
                actual_pos = sensor_data.copy()
                actual_vel_prev = actual_vel.copy()
                actual_vel = (actual_pos - actual_pos_prev) / model.opt.timestep
                actual_acc = (actual_vel - actual_vel_prev) / model.opt.timestep

                data.ctrl[:] = computed_torque_controller.control(desired_pos, sensor_data)

                actual_poses[time_num, :] = sensor_data.copy()
                motor_ctrls[time_num, :] = data.ctrl.copy()
                actual_vels[time_num, :] = actual_vel.copy()
                actual_accs[time_num, :] = actual_acc.copy()

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
            plt.plot(times, desired_vels[:, i], '-', label='desired velocity')
            plt.plot(times, actual_vels[:, i], '--', label='actual velocity')
            plt.axhline(phi[i], color='g', linestyle='-.', label='max velocity')
            plt.axhline(-phi[i], color='g', linestyle='-.', label='max velocity')
            plt.legend()
        plt.tight_layout()

        plt.figure(3)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, desired_accs[:, i], '-', label='desired acceleration')
            plt.plot(times, actual_accs[:, i], '--', label='actual acceleration')
            plt.axhline(alpha[i], color='g', linestyle='-.', label='max acceleration')
            plt.axhline(-alpha[i], color='g', linestyle='-.', label='max acceleration')
            plt.legend()
        plt.tight_layout()

        plt.figure(4)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, motor_ctrls[:, i], '-', label='motor torque')
            plt.axhline(mu[i], color='g', linestyle='-.', label='max torque')
            plt.axhline(-mu[i], color='g', linestyle='-.', label='max torque')
            plt.legend()
        plt.tight_layout()

        plt.show()
