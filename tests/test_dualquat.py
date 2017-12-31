#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from math3d import Quat, DualQuat
import unittest
import math
import numpy as np

class TestDualQuat(unittest.TestCase):
    def test_trans(self):
        q = DualQuat.Trans(t=Quat(1.0, 2.0, 3.0, 0.0), r=Quat.Euler([0.5 * math.pi, 0.0, 0.0]))
        #q = DualQuat.Trans(t=Quat(0.0, 0.0, 0.0, 0.0), r=Quat.Euler([0.0, 0.0, 0.0]))
        v = [1.0, 0.0, 2.0]
        vq = q.transformVec(v)
        np.testing.assert_almost_equal(vq[0], 2)
        np.testing.assert_almost_equal(vq[1], 0)
        np.testing.assert_almost_equal(vq[2], 3)


if __name__ == '__main__':
    unittest.main()
