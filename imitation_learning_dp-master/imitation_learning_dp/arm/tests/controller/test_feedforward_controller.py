from unittest import TestCase

import time
import mujoco
import mujoco.viewer
import numpy as np

from arm.controller import PIDController, FeedforwardController
from matplotlib import pyplot as plt

from arm.robot import *
from arm.motion_planning import *
from spatialmath import SO3, SE3


class TestFeedforwardController(TestCase):
    def test_inv_dynamics(self):
        model = mujoco.MjModel.from_xml_path("../../assets/universal_robots_ur5e/scene.xml")
        data = mujoco.MjData(model)

        robot = UR5e()
        q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        robot.set_joint(q0)

        kp = 1.0
        ki = 0.0
        kd = 0.0

        position_controllers = [PIDController(kp, ki, kd, ts=model.opt.timestep) for _ in range(6)]
        velocity_controllers = [PIDController(kp, ki, kd, ts=model.opt.timestep) for _ in range(6)]

        # 关节6
        Tm = 0.01198
        omega_m = 2 * np.pi / Tm
        km = 50
        kp = 0.45 * km
        ki = kp / (0.85 * Tm)
        kd = kp * np.pi / (4 * omega_m)
        velocity_controllers[5].set_parameter(kp, 0.1 * ki, 0 * kd)

        Tm = 0.01866
        omega_m = 2 * np.pi / Tm
        km = 100
        kp = 0.5 * km
        position_controllers[5].set_parameter(kp, 0, 0)

        ## 关节5
        Tm = 0.01207
        omega_m = 2 * np.pi / Tm
        km = 50
        kp = 0.45 * km
        ki = kp / (0.85 * Tm)
        kd = kp * np.pi / (4 * omega_m)
        velocity_controllers[4].set_parameter(kp, 0.05 * ki, 0 * kd)

        Tm = 0.0192
        omega_m = 2 * np.pi / Tm
        km = 100
        kp = 0.5 * km
        position_controllers[4].set_parameter(kp, 0, 0)

        ## 关节4
        Tm = 0.01207
        omega_m = 2 * np.pi / Tm
        km = 50
        kp = 0.45 * km
        ki = kp / (0.85 * Tm)
        kd = kp * np.pi / (4 * omega_m)
        velocity_controllers[3].set_parameter(kp, 0.05 * ki, 0 * kd)

        Tm = 0.0192
        omega_m = 2 * np.pi / Tm
        km = 150
        kp = 0.5 * km
        position_controllers[3].set_parameter(kp, 0, 0)

        # 关节3
        Tm = 0.01203
        omega_m = 2 * np.pi / Tm
        km = 110
        kp = 0.45 * km
        ki = kp / (0.85 * Tm)
        kd = kp * np.pi / (4 * omega_m)
        velocity_controllers[2].set_parameter(kp, 0.05 * ki, 0 * kd)

        Tm = 0.0192
        omega_m = 2 * np.pi / Tm
        km = 200
        kp = 0.5 * km
        position_controllers[2].set_parameter(kp, 0, 0)

        ## 关节2
        Tm = 0.02958
        omega_m = 2 * np.pi / Tm
        km = 200
        kp = 0.45 * km
        ki = kp / (0.85 * Tm)
        kd = kp * np.pi / (4 * omega_m)
        velocity_controllers[1].set_parameter(kp, 0.02 * ki, 0)

        km = 80
        kp = 0.5 * km
        position_controllers[1].set_parameter(kp, 0, 0)

        ## 关节1
        Tm = 0.01242
        omega_m = 2 * np.pi / Tm
        km = 100
        kp = 0.45 * km
        ki = kp / (0.85 * Tm)
        kd = kp * np.pi / (4 * omega_m)
        velocity_controllers[0].set_parameter(kp, 0.005 * ki, 0)

        km = 100
        kp = 0.5 * km
        position_controllers[0].set_parameter(kp, 0, 0)

        time1 = 2
        q1 = np.array([0.0, 0.0, np.pi / 2, 0.0, -np.pi / 2, 0.0])
        joint_parameter1 = JointParameter(q0, q1)
        cubic_velocity_parameter1 = CubicVelocityParameter(time1)
        trajectory_parameter1 = TrajectoryParameter(joint_parameter1, cubic_velocity_parameter1)
        trajectory_planner1 = TrajectoryPlanner(trajectory_parameter1)

        time2 = 2
        robot.set_joint(q1)
        T1 = robot.get_cartesian()
        t1 = T1.t
        R1 = SO3(T1.R)
        t2 = t1 + np.array([0.0, -0.3, 0.0])
        R2 = SO3(T1.R)
        position_parameter2 = LinePositionParameter(t1, t2)
        attitude_parameter2 = OneAttitudeParameter(R1, R2)
        cartesian_parameter2 = CartesianParameter(position_parameter2, attitude_parameter2)
        cubic_velocity_parameter2 = CubicVelocityParameter(time2)
        trajectory_parameter2 = TrajectoryParameter(cartesian_parameter2, cubic_velocity_parameter2)
        trajectory_planner2 = TrajectoryPlanner(trajectory_parameter2)

        time3 = 2
        t3 = t2 + np.array([0.3, 0.3, -0.3])
        R3 = R2
        position_parameter3 = LinePositionParameter(t2, t3)
        attitude_parameter3 = OneAttitudeParameter(R2, R3)
        cartesian_parameter3 = CartesianParameter(position_parameter3, attitude_parameter3)
        cubic_velocity_parameter3 = CubicVelocityParameter(time3)
        trajectory_parameter3 = TrajectoryParameter(cartesian_parameter3, cubic_velocity_parameter3)
        trajectory_planner3 = TrajectoryPlanner(trajectory_parameter3)

        time4 = 2
        t4 = t3 + np.array([-0.3, 0.3, 0.3])
        R4 = R3
        position_parameter4 = LinePositionParameter(t3, t4)
        attitude_parameter4 = OneAttitudeParameter(R3, R4)
        cartesian_parameter4 = CartesianParameter(position_parameter4, attitude_parameter4)
        cubic_velocity_parameter4 = CubicVelocityParameter(time4)
        trajectory_parameter4 = TrajectoryParameter(cartesian_parameter4, cubic_velocity_parameter4)
        trajectory_planner4 = TrajectoryPlanner(trajectory_parameter4)

        time5 = 2
        t5 = t4 + np.array([-0.6, 0.0, 0.0])
        R5 = R4
        position_parameter5 = LinePositionParameter(t4, t5)
        attitude_parameter5 = OneAttitudeParameter(R4, R5)
        cartesian_parameter5 = CartesianParameter(position_parameter5, attitude_parameter5)
        cubic_velocity_parameter5 = CubicVelocityParameter(time5)
        trajectory_parameter5 = TrajectoryParameter(cartesian_parameter5, cubic_velocity_parameter5)
        trajectory_planner5 = TrajectoryPlanner(trajectory_parameter5)

        time6 = 2
        t6 = t5 + np.array([0.0, -0.8, 0.0])
        R6 = R5
        tc6 = (t5 + t6) / 2.0
        tc6[0] -= 0.2
        tc6[2] -= 0.3
        position_parameter6 = ArcPointPositionParameter(t5, t6, tc6)
        attitude_parameter6 = OneAttitudeParameter(R5, R6)
        cartesian_parameter6 = CartesianParameter(position_parameter6, attitude_parameter6)
        velocity_parameter6 = CubicVelocityParameter(time6)
        trajectory_parameter6 = TrajectoryParameter(cartesian_parameter6, velocity_parameter6)
        trajectory_planner6 = TrajectoryPlanner(trajectory_parameter6)

        time7 = 2
        t7 = t1
        R7 = R1
        position_parameter7 = LinePositionParameter(t6, t7)
        attitude_parameter7 = OneAttitudeParameter(R6, R7)
        cartesian_parameter7 = CartesianParameter(position_parameter7, attitude_parameter7)
        velocity_parameter7 = CubicVelocityParameter(time7)
        trajectory_parameter7 = TrajectoryParameter(cartesian_parameter7, velocity_parameter7)
        trajectory_planner7 = TrajectoryPlanner(trajectory_parameter7)

        time8 = 2

        total_time = time1 + time2 + time3 + time4 + time5 + time6 + time7 + time8
        time_step_num = round(total_time / model.opt.timestep) + 1
        desired_poses = np.zeros((time_step_num, robot.dof))
        desired_vels = np.zeros_like(desired_poses)
        real_poses = np.zeros_like(desired_poses)
        real_pos_prevs = np.zeros_like(desired_poses)
        sensor_datas = np.zeros_like(desired_poses)
        real_vels = np.zeros_like(desired_poses)
        motor_ctrls = np.zeros_like(desired_poses)
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
            if i > 0:
                desired_vels[i, :] = (desired_poses[i, :] - desired_poses[i - 1, :]) / model.opt.timestep

        time_num = 0
        mujoco.mj_resetData(model, data)
        initial_state = np.zeros(mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_PHYSICS))
        mujoco.mj_setState(model, data, initial_state, mujoco.mjtState.mjSTATE_PHYSICS)
        mujoco.mj_forward(model, data)

        sensor_data = data.sensordata.copy()
        real_pos = sensor_data.copy()
        real_pos_prev = real_pos.copy()
        real_vel = (real_pos - real_pos_prev) / model.opt.timestep

        sensor_datas[time_num, :] = sensor_data.copy()
        real_poses[time_num, :] = real_pos.copy()
        real_pos_prevs[time_num, :] = real_pos_prev.copy()
        real_vels[time_num, :] = real_vel.copy()
        motor_ctrls[time_num, :] = data.ctrl.copy()

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time <= total_time:
                time_num += 1

                if time_num >= time_step_num:
                    break

                desired_pos = desired_poses[time_num, :].copy()
                desired_vel = desired_vels[time_num, :].copy()
                sensor_data = data.sensordata.copy()
                real_pos_prev = real_pos.copy()
                real_pos = sensor_data.copy()
                real_vel = (real_pos - real_pos_prev) / model.opt.timestep

                error_pos = desired_pos - sensor_data

                pos_out = [position_controllers[i].control(desired_pos[i], sensor_data[i]) for i in range(6)]
                for i in range(robot.dof):
                    data.ctrl[i] = velocity_controllers[i].control(pos_out[i], real_vel[i])

                sensor_datas[time_num, :] = sensor_data.copy()
                real_poses[time_num, :] = real_pos.copy()
                real_pos_prevs[time_num, :] = real_pos_prev.copy()
                real_vels[time_num, :] = real_vel.copy()
                motor_ctrls[time_num, :] = data.ctrl.copy()
                error_poses[time_num, :] = error_pos.copy()

                mujoco.mj_step(model, data)

                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

                viewer.sync()

        plt.figure(1)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, desired_poses[:, i], '-', label='desired position')
            plt.plot(times, sensor_datas[:, i], '--', label='actual position')
            plt.legend()
        plt.tight_layout()

        plt.figure(2)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, desired_vels[:, i], '-', label='desired velocity')
            plt.plot(times, real_vels[:, i], '--', label='actual velocity')
            plt.legend()
        plt.tight_layout()

        plt.figure(3)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, motor_ctrls[:, i], '-', label='motor torque')
            plt.legend()
        plt.tight_layout()

        plt.figure(4)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, error_poses[:, i], '-', label='position error')
            plt.legend()
        plt.tight_layout()

        real_accs = np.zeros_like(desired_poses)
        for i in range(times.size):
            if i > 0:
                real_accs[i, :] = (real_vels[i, :] - real_vels[i - 1, :]) / model.opt.timestep

        plt.figure(5)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, real_accs[:, i], '-', label='actual acceleration')
            plt.legend()
        plt.tight_layout()

        motor_torq_th = np.zeros_like(desired_poses)
        for i in range(times.size):
            motor_torq_th[i, :] = robot.inv_dynamics(real_poses[i, :], real_vels[i, :], real_accs[i, :])

        plt.figure(6)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, motor_torq_th[:, i], '-', label='theoretical motor torque')
            plt.plot(times, motor_ctrls[:, i], '--', label='motor torque')
            plt.legend()
        plt.tight_layout()

        plt.show()

    def test_pid_control(self):
        model = mujoco.MjModel.from_xml_path("../../assets/universal_robots_ur5e/scene.xml")
        data = mujoco.MjData(model)

        robot = UR5e()
        q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        robot.set_joint(q0)

        kp = 1.0
        ki = 0.0
        kd = 0.0

        position_controllers = [PIDController(kp, ki, kd, ts=model.opt.timestep) for _ in range(6)]
        velocity_controllers = [PIDController(kp, ki, kd, ts=model.opt.timestep) for _ in range(6)]

        # 关节6
        Tm = 0.01198
        omega_m = 2 * np.pi / Tm
        km = 50
        kp = 0.45 * km
        ki = kp / (0.85 * Tm)
        kd = kp * np.pi / (4 * omega_m)
        velocity_controllers[5].set_parameter(kp, 0.1 * ki, 0 * kd)

        Tm = 0.01866
        omega_m = 2 * np.pi / Tm
        km = 100
        kp = 0.5 * km
        position_controllers[5].set_parameter(kp, 0, 0)

        ## 关节5
        Tm = 0.01207
        omega_m = 2 * np.pi / Tm
        km = 50
        kp = 0.45 * km
        ki = kp / (0.85 * Tm)
        kd = kp * np.pi / (4 * omega_m)
        velocity_controllers[4].set_parameter(kp, 0.05 * ki, 0 * kd)

        Tm = 0.0192
        omega_m = 2 * np.pi / Tm
        km = 100
        kp = 0.5 * km
        position_controllers[4].set_parameter(kp, 0, 0)

        ## 关节4
        Tm = 0.01207
        omega_m = 2 * np.pi / Tm
        km = 50
        kp = 0.45 * km
        ki = kp / (0.85 * Tm)
        kd = kp * np.pi / (4 * omega_m)
        velocity_controllers[3].set_parameter(kp, 0.05 * ki, 0 * kd)

        Tm = 0.0192
        omega_m = 2 * np.pi / Tm
        km = 150
        kp = 0.5 * km
        position_controllers[3].set_parameter(kp, 0, 0)

        # 关节3
        Tm = 0.01203
        omega_m = 2 * np.pi / Tm
        km = 110
        kp = 0.45 * km
        ki = kp / (0.85 * Tm)
        kd = kp * np.pi / (4 * omega_m)
        velocity_controllers[2].set_parameter(kp, 0.05 * ki, 0 * kd)

        Tm = 0.0192
        omega_m = 2 * np.pi / Tm
        km = 200
        kp = 0.5 * km
        position_controllers[2].set_parameter(kp, 0, 0)

        ## 关节2
        Tm = 0.02958
        omega_m = 2 * np.pi / Tm
        km = 200
        kp = 0.45 * km
        ki = kp / (0.85 * Tm)
        kd = kp * np.pi / (4 * omega_m)
        velocity_controllers[1].set_parameter(kp, 0.02 * ki, 0)

        km = 80
        kp = 0.5 * km
        position_controllers[1].set_parameter(kp, 0, 0)

        ## 关节1
        Tm = 0.01242
        omega_m = 2 * np.pi / Tm
        km = 100
        kp = 0.45 * km
        ki = kp / (0.85 * Tm)
        kd = kp * np.pi / (4 * omega_m)
        velocity_controllers[0].set_parameter(kp, 0.005 * ki, 0)

        km = 100
        kp = 0.5 * km
        position_controllers[0].set_parameter(kp, 0, 0)

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
        desired_vels = np.zeros_like(desired_poses)
        real_poses = np.zeros_like(desired_poses)
        real_pos_prevs = np.zeros_like(desired_poses)
        sensor_datas = np.zeros_like(desired_poses)
        real_vels = np.zeros_like(desired_poses)
        motor_ctrls = np.zeros_like(desired_poses)
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
            if i > 0:
                desired_vels[i, :] = (desired_poses[i, :] - desired_poses[i - 1, :]) / model.opt.timestep

        time_num = 0
        mujoco.mj_resetData(model, data)
        initial_state = np.zeros(mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_PHYSICS))
        mujoco.mj_setState(model, data, initial_state, mujoco.mjtState.mjSTATE_PHYSICS)
        mujoco.mj_forward(model, data)

        sensor_data = data.sensordata.copy()
        real_pos = sensor_data.copy()
        real_pos_prev = real_pos.copy()
        real_vel = (real_pos - real_pos_prev) / model.opt.timestep

        sensor_datas[time_num, :] = sensor_data.copy()
        real_poses[time_num, :] = real_pos.copy()
        real_pos_prevs[time_num, :] = real_pos_prev.copy()
        real_vels[time_num, :] = real_vel.copy()
        motor_ctrls[time_num, :] = data.ctrl.copy()

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time <= total_time:
                time_num += 1

                if time_num >= time_step_num:
                    break

                desired_pos = desired_poses[time_num, :].copy()
                desired_vel = desired_vels[time_num, :].copy()
                sensor_data = data.sensordata.copy()
                real_pos_prev = real_pos.copy()
                real_pos = sensor_data.copy()
                real_vel = (real_pos - real_pos_prev) / model.opt.timestep

                error_pos = desired_pos - sensor_data

                pos_out = [position_controllers[i].control(desired_pos[i], sensor_data[i]) for i in range(6)]
                for i in range(robot.dof):
                    data.ctrl[i] = velocity_controllers[i].control(pos_out[i], real_vel[i])

                sensor_datas[time_num, :] = sensor_data.copy()
                real_poses[time_num, :] = real_pos.copy()
                real_pos_prevs[time_num, :] = real_pos_prev.copy()
                real_vels[time_num, :] = real_vel.copy()
                motor_ctrls[time_num, :] = data.ctrl.copy()
                error_poses[time_num, :] = error_pos.copy()

                mujoco.mj_step(model, data)

                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

                viewer.sync()

        plt.figure(1)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, desired_poses[:, i], '-', label='desired position')
            plt.plot(times, sensor_datas[:, i], '--', label='actual position')
            plt.legend()
        plt.tight_layout()

        plt.figure(2)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, desired_vels[:, i], '-', label='desired velocity')
            plt.plot(times, real_vels[:, i], '--', label='actual velocity')
            plt.legend()
        plt.tight_layout()

        plt.figure(3)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, motor_ctrls[:, i], '-', label='motor torque')
            plt.legend()
        plt.tight_layout()

        plt.figure(4)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, error_poses[:, i], '-', label='position error')
            plt.legend()
        plt.tight_layout()

        real_accs = np.zeros_like(desired_poses)
        for i in range(times.size):
            if i > 0:
                real_accs[i, :] = (real_vels[i, :] - real_vels[i - 1, :]) / model.opt.timestep

        plt.figure(5)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, real_accs[:, i], '-', label='actual acceleration')
            plt.legend()
        plt.tight_layout()

        motor_torq_th = np.zeros_like(desired_poses)
        for i in range(times.size):
            motor_torq_th[i, :] = robot.inv_dynamics(real_poses[i, :], real_vels[i, :], real_accs[i, :])

        plt.figure(6)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, motor_torq_th[:, i], '-', label='theoretical motor torque')
            plt.plot(times, motor_ctrls[:, i], '--', label='motor torque')
            plt.legend()
        plt.tight_layout()

        plt.show()

    def test_feedforward_control(self):
        model = mujoco.MjModel.from_xml_path("../../assets/universal_robots_ur5e/scene.xml")
        data = mujoco.MjData(model)

        robot = UR5e()
        q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        robot.set_joint(q0)

        kp = 1.0
        ki = 0.0
        kd = 0.0

        position_controllers = [PIDController(kp, ki, kd, ts=model.opt.timestep) for _ in range(6)]
        velocity_controllers = [PIDController(kp, ki, kd, ts=model.opt.timestep) for _ in range(6)]

        # 关节6
        Tm = 0.01198
        omega_m = 2 * np.pi / Tm
        km = 50
        kp = 0.45 * km
        ki = kp / (0.85 * Tm)
        kd = kp * np.pi / (4 * omega_m)
        kp6 = kp
        ki6 = 0.1 * ki
        velocity_controllers[5].set_parameter(kp, 0.1 * ki, 0 * kd)

        Tm = 0.01866
        omega_m = 2 * np.pi / Tm
        km = 100
        kp = 0.5 * km
        position_controllers[5].set_parameter(kp, 0, 0)

        ## 关节5
        Tm = 0.01207
        omega_m = 2 * np.pi / Tm
        km = 50
        kp = 0.45 * km
        ki = kp / (0.85 * Tm)
        kd = kp * np.pi / (4 * omega_m)
        kp5 = kp
        ki5 = 0.05 * ki
        velocity_controllers[4].set_parameter(kp, 0.05 * ki, 0 * kd)

        Tm = 0.0192
        omega_m = 2 * np.pi / Tm
        km = 100
        kp = 0.5 * km
        position_controllers[4].set_parameter(kp, 0, 0)

        ## 关节4
        Tm = 0.01207
        omega_m = 2 * np.pi / Tm
        km = 50
        kp = 0.45 * km
        ki = kp / (0.85 * Tm)
        kd = kp * np.pi / (4 * omega_m)
        kp4 = kp
        ki4 = 0.05 * ki
        velocity_controllers[3].set_parameter(kp, 0.05 * ki, 0 * kd)

        Tm = 0.0192
        omega_m = 2 * np.pi / Tm
        km = 150
        kp = 0.5 * km
        position_controllers[3].set_parameter(kp, 0, 0)

        # 关节3
        Tm = 0.01203
        omega_m = 2 * np.pi / Tm
        km = 110
        kp = 0.45 * km
        ki = kp / (0.85 * Tm)
        kd = kp * np.pi / (4 * omega_m)
        kp3 = kp
        ki3 = 0.05 * ki
        velocity_controllers[2].set_parameter(kp, 0.05 * ki, 0 * kd)

        Tm = 0.0192
        omega_m = 2 * np.pi / Tm
        km = 200
        kp = 0.5 * km
        position_controllers[2].set_parameter(kp, 0, 0)

        ## 关节2
        Tm = 0.02958
        omega_m = 2 * np.pi / Tm
        km = 200
        kp = 0.45 * km
        ki = kp / (0.85 * Tm)
        kd = kp * np.pi / (4 * omega_m)
        kp2 = kp
        ki2 = 0.02 * ki
        velocity_controllers[1].set_parameter(kp, 0.02 * ki, 0)

        km = 80
        kp = 0.5 * km
        position_controllers[1].set_parameter(kp, 0, 0)

        ## 关节1
        Tm = 0.01242
        omega_m = 2 * np.pi / Tm
        km = 100
        kp = 0.45 * km
        ki = kp / (0.85 * Tm)
        kd = kp * np.pi / (4 * omega_m)
        kp1 = kp
        ki1 = 0.005 * ki
        velocity_controllers[0].set_parameter(kp, 0.005 * ki, 0)

        km = 100
        kp = 0.5 * km
        position_controllers[0].set_parameter(kp, 0, 0)

        kps = [kp1, kp2, kp3, kp4, kp5, kp6]
        kis = [ki1, ki2, ki3, ki4, ki5, ki6]
        kds = [0.0 for i in range(robot.dof)]
        velocity_controller = FeedforwardController(kps, kis, kds, robot, ts=model.opt.timestep)

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
        desired_vels = np.zeros_like(desired_poses)
        real_poses = np.zeros_like(desired_poses)
        real_pos_prevs = np.zeros_like(desired_poses)
        sensor_datas = np.zeros_like(desired_poses)
        real_vels = np.zeros_like(desired_poses)
        motor_ctrls = np.zeros_like(desired_poses)
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
            if i > 0:
                desired_vels[i, :] = (desired_poses[i, :] - desired_poses[i - 1, :]) / model.opt.timestep

        time_num = 0
        mujoco.mj_resetData(model, data)
        initial_state = np.zeros(mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_PHYSICS))
        mujoco.mj_setState(model, data, initial_state, mujoco.mjtState.mjSTATE_PHYSICS)
        mujoco.mj_forward(model, data)

        sensor_data = data.sensordata.copy()
        real_pos = sensor_data.copy()
        real_pos_prev = real_pos.copy()
        real_vel = (real_pos - real_pos_prev) / model.opt.timestep

        sensor_datas[time_num, :] = sensor_data.copy()
        real_poses[time_num, :] = real_pos.copy()
        real_pos_prevs[time_num, :] = real_pos_prev.copy()
        real_vels[time_num, :] = real_vel.copy()
        motor_ctrls[time_num, :] = data.ctrl.copy()

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time <= total_time:
                time_num += 1

                if time_num >= time_step_num:
                    break

                desired_pos = desired_poses[time_num, :].copy()
                desired_vel = desired_vels[time_num, :].copy()
                sensor_data = data.sensordata.copy()
                real_pos_prev = real_pos.copy()
                real_pos = sensor_data.copy()
                real_vel = (real_pos - real_pos_prev) / model.opt.timestep

                error_pos = desired_pos - sensor_data

                pos_out = [position_controllers[i].control(desired_pos[i], sensor_data[i]) for i in range(6)]
                data.ctrl[:] = velocity_controller.control(pos_out, real_vel, desired_pos)

                sensor_datas[time_num, :] = sensor_data.copy()
                real_poses[time_num, :] = real_pos.copy()
                real_pos_prevs[time_num, :] = real_pos_prev.copy()
                real_vels[time_num, :] = real_vel.copy()
                motor_ctrls[time_num, :] = data.ctrl.copy()
                error_poses[time_num, :] = error_pos.copy()

                mujoco.mj_step(model, data)

                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

                viewer.sync()

        plt.figure(1)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, desired_poses[:, i], '-', label='desired position')
            plt.plot(times, sensor_datas[:, i], '--', label='actual position')
            plt.legend()
        plt.tight_layout()

        plt.figure(2)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, desired_vels[:, i], '-', label='desired velocity')
            plt.plot(times, real_vels[:, i], '--', label='actual velocity')
            plt.legend()
        plt.tight_layout()

        plt.figure(3)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, motor_ctrls[:, i], '-', label='motor torque')
            plt.legend()
        plt.tight_layout()

        plt.figure(4)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, error_poses[:, i], '-', label='position error')
            plt.legend()
        plt.tight_layout()

        real_accs = np.zeros_like(desired_poses)
        for i in range(times.size):
            if i > 0:
                real_accs[i, :] = (real_vels[i, :] - real_vels[i - 1, :]) / model.opt.timestep

        plt.figure(5)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, real_accs[:, i], '-', label='actual acceleration')
            plt.legend()
        plt.tight_layout()

        motor_torq_th = np.zeros_like(desired_poses)
        for i in range(times.size):
            motor_torq_th[i, :] = robot.inv_dynamics(real_poses[i, :], real_vels[i, :], real_accs[i, :])

        plt.figure(6)
        for i in range(robot.dof):
            plt.subplot(3, 2, i + 1)
            plt.plot(times, motor_torq_th[:, i], '-', label='theoretical motor torque')
            plt.plot(times, motor_ctrls[:, i], '--', label='motor torque')
            plt.legend()
        plt.tight_layout()

        plt.show()