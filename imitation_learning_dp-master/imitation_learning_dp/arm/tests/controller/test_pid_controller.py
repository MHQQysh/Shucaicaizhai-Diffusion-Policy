from unittest import TestCase

import numpy as np
from arm.controller import PIDController
import matplotlib.pyplot as plt


class TestPIDController(TestCase):
    def test_PIDController(self):
        kp = 1.0
        ki = 5.0
        kd = 10.0
        pid_controller = PIDController(kp, ki, kd, ts=0.001, filter_coefficient=10)
        t = np.linspace(0, 10, 10001)
        x = np.ones_like(t)
        y = np.zeros_like(t)

        for i in range(10001):
            y[i] = pid_controller.control(x[i], 0)
        print(x)
        print(y)
        plt.figure(1)
        plt.plot(t, x)
        plt.plot(t, y)
        plt.show()
