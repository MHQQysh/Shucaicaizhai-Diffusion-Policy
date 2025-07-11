from unittest import TestCase

import numpy as np
from spatialmath import SE3, SO3
import mujoco
import mujoco.viewer

from arm.robot import IIWA14
from arm.motion_planning import *


class TestIIWA(TestCase):
    def test_motion(self):
        model = mujoco.MjModel.from_xml_path("../../assets/kuka_iiwa_14/scene.xml")
        data = mujoco.MjData(model)

        total_time = 1000.0

        mujoco.mj_setState(model, data, model.key_qpos[0, :] * 0, mujoco.mjtState.mjSTATE_QPOS)
        mujoco.mj_forward(model, data)

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time <= total_time:
                data.ctrl[:] = model.key_qpos[0, :] * 0
                data.ctrl[6] = 1

                mujoco.mj_step(model, data)

                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

                viewer.sync()

    def test_fkine(self):
        model = mujoco.MjModel.from_xml_path("../../assets/kuka_iiwa_14/scene.xml")
        data = mujoco.MjData(model)

        q0 = np.random.random(7)

        mujoco.mj_setState(model, data, q0, mujoco.mjtState.mjSTATE_QPOS)
        mujoco.mj_forward(model, data)

        print("xmat: ")
        print(np.reshape(data.site_xmat[0], (3, 3)))
        print("xpos: ")
        print(data.site_xpos[0])

        robot = IIWA14()
        T = robot.fkine(q0)
        print("T: ")
        print(T)
        qe = robot.ikine(T)
        print("qe: ")
        print(qe)
        Te = robot.fkine(qe)
        print("Te: ")
        print(Te)

    def test_motion2(self):
        model = mujoco.MjModel.from_xml_path("../../assets/kuka_iiwa_14/scene.xml")
        data = mujoco.MjData(model)

        robot = IIWA14(tool=np.array([0, 0, 0.2]))
        robot.phi = 0.0
        T0 = SE3.Trans(0.6, -0.5, 0.4) * SE3.Ry(np.pi * 2 / 3)
        q0 = robot.ikine(T0)

        total_time = 20.0

        mujoco.mj_setState(model, data, q0, mujoco.mjtState.mjSTATE_QPOS)
        mujoco.mj_forward(model, data)

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time <= total_time:
                phi = np.pi * np.sin(data.time / 1.0)
                robot.phi = phi
                robot.move_cartesian(T0)

                data.qpos[:] = robot.get_joint()

                mujoco.mj_step(model, data)

                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

                viewer.sync()

    def test_motion3(self):
        model = mujoco.MjModel.from_xml_path("../../assets/kuka_iiwa_14/scene.xml")
        data = mujoco.MjData(model)

        q0 = np.zeros(7)
        robot = IIWA14(tool=np.array([0, 0, 0.2]))
        robot.phi = np.pi

        motion_time = 5.0

        time1 = motion_time
        T1 = SE3.Trans(-0.5, -0.5, 0.4) * SE3.Rx(np.pi)
        q1 = robot.ikine(T1)
        joint_parameter1 = JointParameter(q0, q1)
        velocity_parameter1 = QuinticVelocityParameter(time1)
        trajectory_parameter1 = TrajectoryParameter(joint_parameter1, velocity_parameter1)
        trajectory_planner1 = TrajectoryPlanner(trajectory_parameter1)

        time2 = motion_time
        t1 = T1.t
        R1 = SO3(T1.R)
        t2 = t1 + np.array([1.0, 0.0, 0.0])
        R2 = R1
        position_parameter2 = LinePositionParameter(t1, t2)
        attitude_parameter2 = OneAttitudeParameter(R1, R2)
        cartesian_parameter2 = CartesianParameter(position_parameter2, attitude_parameter2)
        velocity_parameter2 = QuinticVelocityParameter(time2)
        trajectory_parameter2 = TrajectoryParameter(cartesian_parameter2, velocity_parameter2)
        trajectory_planner2 = TrajectoryPlanner(trajectory_parameter2)

        time3 = motion_time
        t3 = t2 + np.array([0.0, 1.0, 0.0])
        R3 = R2
        position_parameter3 = LinePositionParameter(t2, t3)
        attitude_parameter3 = OneAttitudeParameter(R2, R3)
        cartesian_parameter3 = CartesianParameter(position_parameter3, attitude_parameter3)
        velocity_parameter3 = QuinticVelocityParameter(time3)
        trajectory_parameter3 = TrajectoryParameter(cartesian_parameter3, velocity_parameter3)
        trajectory_planner3 = TrajectoryPlanner(trajectory_parameter3)

        time4 = motion_time
        t4 = t3 + np.array([-1.0, 0.0, 0.0])
        R4 = R3
        position_parameter4 = LinePositionParameter(t3, t4)
        attitude_parameter4 = OneAttitudeParameter(R3, R4)
        cartesian_parameter4 = CartesianParameter(position_parameter4, attitude_parameter4)
        velocity_parameter4 = QuinticVelocityParameter(time4)
        trajectory_parameter4 = TrajectoryParameter(cartesian_parameter4, velocity_parameter4)
        trajectory_planner4 = TrajectoryPlanner(trajectory_parameter4)

        time5 = motion_time
        t5 = t4 + np.array([0.0, -1.0, 0.0])
        R5 = R4
        position_parameter5 = LinePositionParameter(t4, t5)
        attitude_parameter5 = OneAttitudeParameter(R4, R5)
        cartesian_parameter5 = CartesianParameter(position_parameter5, attitude_parameter5)
        velocity_parameter5 = QuinticVelocityParameter(time5)
        trajectory_parameter5 = TrajectoryParameter(cartesian_parameter5, velocity_parameter5)
        trajectory_planner5 = TrajectoryPlanner(trajectory_parameter5)

        total_time = time1 + time2 + time3 + time4 + time5 + motion_time
        time_step_num = round(total_time / model.opt.timestep) + 1
        times = np.linspace(0, total_time, time_step_num)
        desired_poses = np.zeros((time_step_num, robot.dof))

        joint_position = np.zeros(7)
        for i, timei in enumerate(times):
            if timei <= time1:
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
                planner_interpolate = trajectory_planner5.interpolate(timei - time1 - time2 - time3 - time5)
                robot.move_cartesian(planner_interpolate)
                joint_position = robot.get_joint()

            desired_poses[i, :] = joint_position

        time_num = 0
        mujoco.mj_resetData(model, data)
        mujoco.mj_setState(model, data, q0, mujoco.mjtState.mjSTATE_QPOS)
        mujoco.mj_forward(model, data)

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time <= total_time:
                time_num += 1
                if time_num >= time_step_num:
                    break

                data.qpos[:] = desired_poses[time_num, :]

                mujoco.mj_step(model, data)

                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

                viewer.sync()
