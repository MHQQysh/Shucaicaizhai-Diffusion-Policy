from unittest import TestCase

import numpy as np
from spatialmath import SO3
from arm.geometry.rotation.SO3Impl import SO3Impl


class TestSO3Impl(TestCase):
    def test_add(self):
        print('******test add******')
        R0 = SO3().RPY(0.0, 0.0, np.pi)
        R1 = SO3().RPY(np.pi, 0.0, 0.0)
        print(R0)
        print(R1)
        R = R0 + R1
        print(R)

        R0_impl = SO3Impl().RPY(0.0, 0.0, np.pi)
        R1_impl = SO3Impl().RPY(np.pi, 0.0, 0.0)
        print(R0_impl)
        print(R1_impl)
        R_impl = R0_impl + R1
        print(R_impl)

    def test_sub(self):
        print('******test sub******')
        R0 = SO3().RPY(0.0, 0.0, np.pi)
        R1 = SO3().RPY(np.pi, 0.0, 0.0)
        print(R0)
        print(R1)
        R = R0 - R1
        print(R)

        R0_impl = SO3Impl().RPY(0.0, 0.0, np.pi)
        R1_impl = SO3Impl().RPY(np.pi, 0.0, 0.0)
        print(R0_impl)
        print(R1_impl)
        R_impl = R0_impl - R1
        print(R_impl)

    def test_mul(self):
        print('******test mul******')
        R0 = SO3().RPY(0.0, 0.0, np.pi)
        R1 = SO3().RPY(np.pi, 0.0, 0.0)
        # print(R0)
        # print(R1)
        R = R0 * R1
        print(R)

        R0_impl = SO3Impl().RPY(0.0, 0.0, np.pi)
        R1_impl = SO3Impl().RPY(np.pi, 0.0, 0.0)
        # print(R0_impl)
        # print(R1_impl)
        R_impl = R0_impl * R1_impl
        print(R_impl)

    def test_mul2(self):
        print('******test mul2******')
        R0_impl = SO3Impl().RPY(0.0, 0.0, np.pi)
        R_impl = 0 * R0_impl
        print(R_impl)

    def test_log(self):
        print('******test log******')
        R0: SO3 = SO3Impl().RPY(np.pi / 3, np.pi / 4, np.pi / 6)
        R0_log = R0.log()
        R0_ = SO3.Exp(R0_log)
        print('R0: ', R0)
        print('R0_log: ', R0.log())
        print('R0_: ', R0_)

    def test_s(self):
        R0 = SO3Impl().RPY(np.pi / 3, np.pi / 4, np.pi / 6)
        R1 = SO3Impl().RPY(np.pi / 4, np.pi / 6, np.pi / 3)

        s = 1.0
        R = R0 + s * (R1 - R0)
        print('R0: ', R0)
        print('R1: ', R1)
        print('R: ', R)
