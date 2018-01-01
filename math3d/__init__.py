from quat import Quat
from dualquat import DualQuat
from trans import Trans

import numpy as np

def crossMat(v):
    x, y, z = v
    return np.array([[0.0, -z, y],
                     [z, 0.0, -x],
                     [-y, x, 0.0]])
