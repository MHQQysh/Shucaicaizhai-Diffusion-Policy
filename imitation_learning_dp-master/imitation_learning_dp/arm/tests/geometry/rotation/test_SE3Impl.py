from unittest import TestCase

import numpy as np
from spatialmath import SE3
from arm.geometry.rotation.SE3Impl import SE3Impl


class TestSE3Impl(TestCase):
    def test_add(self):
        print('******test add******')
        T0 = SE3().RPY(0.0, 0.0, np.pi)
        T1 = SE3().RPY(np.pi, 0.0, 0.0)
        print(T0)
        print(T1)
        R = T0 + T1
        print(R)

        T0_impl = SE3Impl().RPY(0.0, 0.0, np.pi)
        T1_impl = SE3Impl().RPY(np.pi, 0.0, 0.0)
        print(T0_impl)
        print(T1_impl)
        T_impl = T0_impl + T1_impl
        print(T_impl)

    def test_sub(self):
        print('******test sub******')
        T0 = SE3().RPY(0.0, 0.0, np.pi)
        T1 = SE3().RPY(np.pi, 0.0, 0.0)
        print(T0)
        print(T1)
        T = T0 - T1
        print('T: ', T)

        T0_impl = SE3Impl().RPY(0.0, 0.0, np.pi)
        T1_impl = SE3Impl().RPY(np.pi, 0.0, 0.0)
        print(T0_impl)
        print(T1_impl)
        T_impl = T0_impl - T1
        print('T_impl: ', T_impl)

    def test_mul(self):
        print('******test mul******')
        T0 = SE3().RPY(0.0, 0.0, np.pi)
        T1 = SE3().RPY(np.pi, 0.0, 0.0)
        # print(R0)
        # print(R1)
        T = T0 * T1
        print(T)

        T0_impl = SE3Impl().RPY(0.0, 0.0, np.pi)
        T1_impl = SE3Impl().RPY(np.pi, 0.0, 0.0)
        print(T0_impl)
        print(T1_impl)
        T_impl = T0_impl * T1_impl
        print(T_impl)

    def test_mul2(self):
        print('******test mul2******')
        T0_impl = SE3Impl.Trans(1.0, 2.0, 3.0) * SE3Impl().RPY(0.0, 0.0, np.pi)
        T_impl = 0.0 * T0_impl
        print(T_impl)

    def test_s(self):
        T0 = SE3Impl.Trans(0.0, 0.0, 0.0) * SE3Impl().RPY(np.pi / 3, np.pi / 4, np.pi / 6)
        T1 = SE3Impl.Trans(1.0, 2.0, 3.0) * SE3Impl().RPY(np.pi / 4, np.pi / 6, np.pi / 3)

        s = 0.0
        T = T0 + s * (T1 - T0)
        print('T0: ', T0)
        print('T1: ', T1)
        print('T: ', T)
