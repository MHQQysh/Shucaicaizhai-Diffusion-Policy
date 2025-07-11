from unittest import TestCase

import matplotlib.pyplot as plt
import numpy as np
from spatialmath import SE3, SO3
import mujoco
import mujoco.viewer

from arm.robot import IIWA14
from arm.motion_planning import *


class TestForceSensor(TestCase):
    def test_force_sensor(self):
        model = mujoco.MjModel.from_xml_path("../../assets/kuka_iiwa_14/scene.xml")
        data = mujoco.MjData(model)

        q0 = np.zeros(7)
        robot = IIWA14(tool=np.array([0, 0, 0.2]))
        robot.phi = np.pi

        motion_time = 5

        time1 = motion_time
        q1 = np.array([0.0, np.pi / 4, 0.0, -np.pi / 4, 0.0, 0.0, 0.0])
        joint_parameter1 = JointParameter(q0, q1)
        velocity_parameter1 = QuinticVelocityParameter(time1)
        trajectory_parameter1 = TrajectoryParameter(joint_parameter1, velocity_parameter1)
        trajectory_planner1 = TrajectoryPlanner(trajectory_parameter1)

        time2 = motion_time
        q2 = q1
        joint_parameter2 = JointParameter(q1, q2)
        velocity_parameter2 = QuinticVelocityParameter(time2)
        trajectory_parameter2 = TrajectoryParameter(joint_parameter2, velocity_parameter2)
        trajectory_planner2 = TrajectoryPlanner(trajectory_parameter2)

        time3 = motion_time
        q3 = q2
        joint_parameter3 = JointParameter(q2, q3)
        velocity_parameter3 = QuinticVelocityParameter(time3)
        trajectory_parameter3 = TrajectoryParameter(joint_parameter3, velocity_parameter3)
        trajectory_planner3 = TrajectoryPlanner(trajectory_parameter3)

        time4 = motion_time
        q4 = q3
        joint_parameter4 = JointParameter(q3, q4)
        velocity_parameter4 = QuinticVelocityParameter(time4)
        trajectory_parameter4 = TrajectoryParameter(joint_parameter4, velocity_parameter4)
        trajectory_planner4 = TrajectoryPlanner(trajectory_parameter4)

        time5 = motion_time
        q5 = q4
        joint_parameter5 = JointParameter(q4, q5)
        velocity_parameter5 = QuinticVelocityParameter(time5)
        trajectory_parameter5 = TrajectoryParameter(joint_parameter5, velocity_parameter5)
        trajectory_planner5 = TrajectoryPlanner(trajectory_parameter5)

        total_time = time1 + time2 + time3 + time4 + time5 + motion_time
        time_step_num = round(total_time / model.opt.timestep) + 1
        times = np.linspace(0, total_time, time_step_num)
        desired_poses = np.zeros((time_step_num, robot.dof))
        actual_poses = np.zeros((time_step_num, robot.dof))
        actual_vels = np.zeros((time_step_num, robot.dof))
        actual_accs = np.zeros((time_step_num, robot.dof))
        actual_torqs = np.zeros((time_step_num, robot.dof))

        joint_position = np.zeros(7)
        for i, timei in enumerate(times):
            if timei <= time1:
                planner_interpolate = trajectory_planner1.interpolate(timei)
                joint_position = planner_interpolate
                robot.move_joint(joint_position)
            elif timei <= time1 + time2:
                planner_interpolate = trajectory_planner2.interpolate(timei - time1)
                joint_position = planner_interpolate
                robot.move_joint(joint_position)
            elif timei <= time1 + time2 + time3:
                planner_interpolate = trajectory_planner3.interpolate(timei - time1 - time2)
                joint_position = planner_interpolate
                robot.move_joint(joint_position)
            elif timei <= time1 + time2 + time3 + time4:
                planner_interpolate = trajectory_planner4.interpolate(timei - time1 - time2 - time3)
                joint_position = planner_interpolate
                robot.move_joint(joint_position)
            elif timei <= time1 + time2 + time3 + time4 + time5:
                planner_interpolate = trajectory_planner5.interpolate(timei - time1 - time2 - time3 - time4)
                joint_position = planner_interpolate
                robot.move_joint(joint_position)
            desired_poses[i, :] = joint_position

        time_num = 0
        mujoco.mj_setState(model, data, q0, mujoco.mjtState.mjSTATE_QPOS)
        mujoco.mj_forward(model, data)
        actual_poses[time_num, :] = data.qpos[:]
        actual_vels[time_num, :] = data.qvel[:]
        actual_accs[time_num, :] = data.qacc[:]
        actual_torqs[time_num, :] = data.actuator_force[:]

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time <= total_time:

                data.ctrl[:] = desired_poses[time_num, :]

                mujoco.mj_step(model, data)

                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

                viewer.sync()

                time_num += 1
                if time_num >= time_step_num:
                    break

                actual_poses[time_num, :] = data.qpos[:]
                actual_vels[time_num, :] = data.qvel[:]
                actual_accs[time_num, :] = data.qacc[:]
                actual_torqs[time_num, :] = data.actuator_force[:]
