from unittest import TestCase

import numpy as np
from spatialmath import SE3, SO3
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer

from arm.robot import IIWA14
from arm.compliance_control import AdmittanceController
from arm.motion_planning import *


class TestAdmittanceController(TestCase):
    def test_admittance_controller(self):
        m = 1.0
        c = 10.0
        k = 1.0
        dt = 0.001

        admittance_controller = AdmittanceController(m, c, k, dt)

        times = np.linspace(0, 10, 10001)
        print(times)

        traj = np.zeros((times.shape[0], 3))

        for i, timei in enumerate(times):
            u = np.array([0, 0, 1])
            # admittance_controller.update(u)
            # traj[i, :] = admittance_controller.get_state()
            traj[i, :] = admittance_controller.get_position(u)

        plt.figure(1)
        for i in range(3):
            plt.subplot(3, 1, i + 1)
            plt.plot(times, traj[:, i])
        plt.tight_layout()

        plt.show()

    def test_admittance_controller1(self):
        model = mujoco.MjModel.from_xml_path("../../../assets/kuka_iiwa_14/scene.xml")
        data = mujoco.MjData(model)

        q0 = np.array([0.0, np.pi / 4, 0.0, -np.pi / 4, 0.0, np.pi / 2, 0.0])

        m = 100.0
        c = 500.0
        k = 100.0
        admittance_controller = AdmittanceController(m, c, k, model.opt.timestep)

        robot = IIWA14(tool=np.array([0, 0, 0.21]))
        robot.set_joint(q0)
        T0 = robot.fkine(q0)

        time0 = 5

        mujoco.mj_resetData(model, data)
        mujoco.mj_setState(model, data, np.hstack((q0, np.array([0]))), mujoco.mjtState.mjSTATE_QPOS)
        mujoco.mj_forward(model, data)

        g = np.array([0.0, 0.0, -9.81])

        m_tool = 1.0
        delta_x = np.zeros(3)

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():

                if data.time > time0:

                    force_sensor = -data.sensor("force_sensor").data
                    Tt = robot.get_cartesian()
                    Rt = Tt.R
                    force_tool = Rt.T @ (m_tool * g)

                    delta_x = admittance_controller.get_position(Rt @ (force_sensor - force_tool))

                else:
                    delta_x = np.zeros(3)
                Te = SE3.Trans(delta_x) * T0
                robot.move_cartesian(Te)
                qe = robot.get_joint()

                data.ctrl[:] = qe

                mujoco.mj_step(model, data)

                viewer.sync()

    def test_admittance_controller2(self):
        model = mujoco.MjModel.from_xml_path("../../../assets/kuka_iiwa_14/scene.xml")
        data = mujoco.MjData(model)

        q0 = np.array([0.0, np.pi / 4, 0.0, -np.pi / 4, 0.0, np.pi / 2, 0.0])

        m = 100.0
        c = 1000.0
        k = 10 * 0
        admittance_controller = AdmittanceController(m, c, k, model.opt.timestep)
        admittance_controller.f0 = np.array([0, 0, 30, 0, 0, 0])

        robot = IIWA14(tool=np.array([0, 0, 0.21]))
        robot.set_joint(q0)
        T0 = robot.fkine(q0)

        motion_time = 5.0

        time0 = motion_time

        time1 = motion_time
        t0 = T0.t
        R0 = SO3.Ry(np.pi)
        t1 = t0 + np.array([-0.15, 0.0, 0.0])
        R1 = R0
        position_parameter1 = LinePositionParameter(t0, t1)
        attitude_parameter1 = OneAttitudeParameter(R0, R1)
        cartesian_parameter1 = CartesianParameter(position_parameter1, attitude_parameter1)
        velocity_parameter1 = QuinticVelocityParameter(time1)
        trajectory_parameter1 = TrajectoryParameter(cartesian_parameter1, velocity_parameter1)
        trajectory_planner1 = TrajectoryPlanner(trajectory_parameter1)

        time2 = motion_time
        t2 = t1 + np.array([0.15, 0.0, 0.0])
        R2 = R1
        position_parameter2 = LinePositionParameter(t1, t2)
        attitude_parameter2 = OneAttitudeParameter(R1, R2)
        cartesian_parameter2 = CartesianParameter(position_parameter2, attitude_parameter2)
        velocity_parameter2 = QuinticVelocityParameter(time2)
        trajectory_parameter2 = TrajectoryParameter(cartesian_parameter2, velocity_parameter2)
        trajectory_planner2 = TrajectoryPlanner(trajectory_parameter2)

        time3 = motion_time
        t3 = t2 + np.array([-0.15, 0.0, 0.0])
        R3 = R2
        position_parameter3 = LinePositionParameter(t2, t3)
        attitude_parameter3 = OneAttitudeParameter(R2, R3)
        cartesian_parameter3 = CartesianParameter(position_parameter3, attitude_parameter3)
        velocity_parameter3 = QuinticVelocityParameter(time3)
        trajectory_parameter3 = TrajectoryParameter(cartesian_parameter3, velocity_parameter3)
        trajectory_planner3 = TrajectoryPlanner(trajectory_parameter3)

        time4 = motion_time
        t4 = t3 + np.array([0.15, 0.0, 0.0])
        R4 = R3
        position_parameter4 = LinePositionParameter(t3, t4)
        attitude_parameter4 = OneAttitudeParameter(R3, R4)
        cartesian_parameter4 = CartesianParameter(position_parameter4, attitude_parameter4)
        velocity_parameter4 = QuinticVelocityParameter(time4)
        trajectory_parameter4 = TrajectoryParameter(cartesian_parameter4, velocity_parameter4)
        trajectory_planner4 = TrajectoryPlanner(trajectory_parameter4)

        time5 = motion_time
        t5 = t4 + np.array([-0.15, 0.0, 0.0])
        R5 = R4
        position_parameter5 = LinePositionParameter(t4, t5)
        attitude_parameter5 = OneAttitudeParameter(R4, R5)
        cartesian_parameter5 = CartesianParameter(position_parameter5, attitude_parameter5)
        velocity_parameter5 = QuinticVelocityParameter(time5)
        trajectory_parameter5 = TrajectoryParameter(cartesian_parameter5, velocity_parameter5)
        trajectory_planner5 = TrajectoryPlanner(trajectory_parameter5)

        time6 = motion_time
        t6 = t5 + np.array([0.15, 0.0, 0.0])
        R6 = R5
        position_parameter6 = LinePositionParameter(t5, t6)
        attitude_parameter6 = OneAttitudeParameter(R5, R6)
        cartesian_parameter6 = CartesianParameter(position_parameter6, attitude_parameter6)
        velocity_parameter6 = QuinticVelocityParameter(time6)
        trajectory_parameter6 = TrajectoryParameter(cartesian_parameter6, velocity_parameter6)
        trajectory_planner6 = TrajectoryPlanner(trajectory_parameter6)

        times = np.array([time0, time1, time2, time3, time4, time5, time6])
        trajectory_planners = [trajectory_planner1, trajectory_planner2, trajectory_planner3, trajectory_planner4,
                               trajectory_planner5, trajectory_planner6]

        mujoco.mj_resetData(model, data)
        mujoco.mj_setState(model, data, np.hstack((q0, np.array([0]))), mujoco.mjtState.mjSTATE_QPOS)
        mujoco.mj_forward(model, data)

        g = np.array([0.0, 0.0, -9.81])

        m_tool = 1.0
        delta_x = np.zeros(3)

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():

                if data.time > time0:

                    for i in range(1, times.size):
                        if data.time < np.sum(times[: i + 1]):
                            T0 = trajectory_planners[i - 1].interpolate(data.time - np.sum(times[: i]))
                            break

                    force_sensor = -data.sensor("force_sensor").data
                    Tt = robot.get_cartesian()
                    Rt = Tt.R
                    force_tool = Rt.T @ (m_tool * g)

                    delta_x = admittance_controller.get_position(Rt @ (force_sensor - force_tool))

                else:
                    delta_x = np.zeros(3)
                Te = SE3.Trans(delta_x) * T0
                robot.move_cartesian(Te)
                qe = robot.get_joint()

                data.ctrl[:] = qe

                mujoco.mj_step(model, data)

                viewer.sync()

    def test_admittance_controller3(self):
        model = mujoco.MjModel.from_xml_path("../../../assets/kuka_iiwa_14/scene.xml")
        data = mujoco.MjData(model)

        q0 = np.array([0.0, np.pi / 4, 0.0, np.pi / 4, 0.0, np.pi / 2, 0.0])

        m = 100.0
        c = 1000.0
        k = 1000.0 * 0
        admittance_controller = AdmittanceController(m, c, k, model.opt.timestep)
        admittance_controller.f0 = np.array([-15, 0.0, 0.0, 0.0, 0.0, 0.0])

        robot = IIWA14(tool=np.array([0, 0, 0.21]))
        robot.set_joint(q0)
        T0 = robot.fkine(q0)

        motion_time = 5.0

        time0 = motion_time

        time1 = motion_time
        t0 = T0.t
        R0 = SO3.Ry(np.pi / 2)
        t1 = t0 + np.array([0.0, 0.0, -0.15])
        R1 = R0
        position_parameter1 = LinePositionParameter(t0, t1)
        attitude_parameter1 = OneAttitudeParameter(R0, R1)
        cartesian_parameter1 = CartesianParameter(position_parameter1, attitude_parameter1)
        velocity_parameter1 = QuinticVelocityParameter(time1)
        trajectory_parameter1 = TrajectoryParameter(cartesian_parameter1, velocity_parameter1)
        trajectory_planner1 = TrajectoryPlanner(trajectory_parameter1)

        time2 = motion_time
        t2 = t1 + np.array([0.0, 0.0, 0.15])
        R2 = R1
        position_parameter2 = LinePositionParameter(t1, t2)
        attitude_parameter2 = OneAttitudeParameter(R1, R2)
        cartesian_parameter2 = CartesianParameter(position_parameter2, attitude_parameter2)
        velocity_parameter2 = QuinticVelocityParameter(time2)
        trajectory_parameter2 = TrajectoryParameter(cartesian_parameter2, velocity_parameter2)
        trajectory_planner2 = TrajectoryPlanner(trajectory_parameter2)

        time3 = motion_time
        t3 = t2 + np.array([0.0, 0.0, -0.15])
        R3 = R2
        position_parameter3 = LinePositionParameter(t2, t3)
        attitude_parameter3 = OneAttitudeParameter(R2, R3)
        cartesian_parameter3 = CartesianParameter(position_parameter3, attitude_parameter3)
        velocity_parameter3 = QuinticVelocityParameter(time3)
        trajectory_parameter3 = TrajectoryParameter(cartesian_parameter3, velocity_parameter3)
        trajectory_planner3 = TrajectoryPlanner(trajectory_parameter3)

        time4 = motion_time
        t4 = t3 + np.array([0.0, 0.0, 0.15])
        R4 = R3
        position_parameter4 = LinePositionParameter(t3, t4)
        attitude_parameter4 = OneAttitudeParameter(R3, R4)
        cartesian_parameter4 = CartesianParameter(position_parameter4, attitude_parameter4)
        velocity_parameter4 = QuinticVelocityParameter(time4)
        trajectory_parameter4 = TrajectoryParameter(cartesian_parameter4, velocity_parameter4)
        trajectory_planner4 = TrajectoryPlanner(trajectory_parameter4)

        time5 = motion_time
        t5 = t4 + np.array([0.0, 0.0, -0.15])
        R5 = R4
        position_parameter5 = LinePositionParameter(t4, t5)
        attitude_parameter5 = OneAttitudeParameter(R4, R5)
        cartesian_parameter5 = CartesianParameter(position_parameter5, attitude_parameter5)
        velocity_parameter5 = QuinticVelocityParameter(time5)
        trajectory_parameter5 = TrajectoryParameter(cartesian_parameter5, velocity_parameter5)
        trajectory_planner5 = TrajectoryPlanner(trajectory_parameter5)

        time6 = motion_time
        t6 = t5 + np.array([0.0, 0.0, 0.15])
        R6 = R5
        position_parameter6 = LinePositionParameter(t5, t6)
        attitude_parameter6 = OneAttitudeParameter(R5, R6)
        cartesian_parameter6 = CartesianParameter(position_parameter6, attitude_parameter6)
        velocity_parameter6 = QuinticVelocityParameter(time6)
        trajectory_parameter6 = TrajectoryParameter(cartesian_parameter6, velocity_parameter6)
        trajectory_planner6 = TrajectoryPlanner(trajectory_parameter6)

        times = np.array([time0, time1, time2, time3, time4, time5, time6])
        trajectory_planners = [trajectory_planner1, trajectory_planner2, trajectory_planner3, trajectory_planner4,
                               trajectory_planner5, trajectory_planner6]

        mujoco.mj_resetData(model, data)
        mujoco.mj_setState(model, data, np.hstack((q0, np.array([0]))), mujoco.mjtState.mjSTATE_QPOS)
        mujoco.mj_forward(model, data)

        g = np.array([0.0, 0.0, -9.81])

        m_tool = 1.0
        delta_x = np.zeros(3)

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():

                if data.time > time0:

                    for i in range(1, times.size):
                        if data.time < np.sum(times[: i + 1]):
                            T0 = trajectory_planners[i - 1].interpolate(data.time - np.sum(times[: i]))
                            break

                    force_sensor = -data.sensor("force_sensor").data
                    Tt = robot.get_cartesian()
                    Rt = Tt.R
                    force_tool = Rt.T @ (m_tool * g)

                    delta_x = admittance_controller.get_position(Rt @ (force_sensor - force_tool))

                else:
                    delta_x = np.zeros(3)
                Te = SE3.Trans(delta_x) * T0
                robot.move_cartesian(Te)
                qe = robot.get_joint()

                data.ctrl[:] = qe

                mujoco.mj_step(model, data)

                viewer.sync()
