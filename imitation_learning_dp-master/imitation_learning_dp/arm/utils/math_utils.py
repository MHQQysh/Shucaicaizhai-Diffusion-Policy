import numpy as np
from arm.constanst import MathConst


class MathUtils:
    @staticmethod
    def near_zero(value: float) -> bool:
        return np.abs(value) < MathConst.EPS
