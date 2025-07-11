from unittest import TestCase

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from arm.robot import UR5e


class TestRobot(TestCase):
    def test_get_geometry(self):
        q = [0.0, 0.0, np.pi / 2, 0.0, -np.pi / 2, 0.0]
        robot = UR5e()
        fkine = robot.fkine(q)
        print(fkine)
        robot.set_joint(q)
        geometries = robot.get_geometries()
        fig = plt.figure(1)
        ax = plt.axes(projection='3d')
        for geometry in geometries:
            geometry.plot(ax)

        ax.set_xlim([-1.0, 1.0])
        ax.set_ylim([-1.0, 1.0])
        ax.set_zlim([-1.0, 1.0])

        ax.set_box_aspect([1, 1, 1])

        plt.show()

    def test_ikine(self):
        robot = UR5e()
        q = (np.random.random(6) - 0.5) * 100

        q0 = [q[i] for i in range(6)]
        robot.set_joint(q0)
        T0 = robot.fkine(q)
        print(T0)

        qe = robot.ikine(T0)
        print(robot.fkine(qe))
        print(q)
        print(qe)

    def test_inv_dynamics(self):
        robot = UR5e()

        qs = np.zeros(robot.dof)
        dqs = np.zeros(robot.dof)
        ddqs = np.zeros(robot.dof)
        ddqs[5] = 1

        torques = robot.inv_dynamics(qs, dqs, ddqs)
        print(torques)

    def test_cal_adaptive_identification_matrix(self):
        robot = UR5e()

        qs = np.zeros(robot.dof)
        dqs = np.zeros(robot.dof)
        dqrs = np.zeros(robot.dof)
        ddqs = np.zeros(robot.dof)
        # qs[4] = np.pi / 2

        qs = np.random.random(robot.dof)
        dqs = np.random.random(robot.dof)
        ddqs = np.random.random(robot.dof)

        torques = robot.inv_dynamics(qs, dqs, ddqs)
        print(torques)

        torques2 = robot.get_identification_matrix(qs, dqs, ddqs)
        temp2 = robot.inertial_parameters
        print(torques2 @ temp2)

        torques3 = robot.get_adaptive_identification_matrix(qs, dqs, dqs, ddqs)
        temp3 = robot.inertial_parameters
        # print(torques3)
        print(torques3 @ temp3)

        parameters = np.linspace(0, 65, 66)
        robot.inertial_parameters = robot.inertial_parameters * 10
        # robot.inertial_parameters = parameters
        print(robot.inertial_parameters)

        torques = robot.inv_dynamics(qs, dqs, ddqs)
        print(torques)

        torques2 = robot.get_identification_matrix(qs, dqs, ddqs)
        temp2 = robot.inertial_parameters
        print(torques2 @ temp2)

        torques3 = robot.get_adaptive_identification_matrix(qs, dqs, 2 * dqs, ddqs)
        temp3 = robot.inertial_parameters
        # print(torques3)
        print(torques3 @ temp3)

        torques4 = robot.inv_dynamics_adaptive(qs, dqs, 2 * dqs, ddqs)
        print(torques4)
