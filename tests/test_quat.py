#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from math3d import Quat
import unittest
import math
import numpy as np

class TestQuat(unittest.TestCase):
    def test_quat(self):
        (x, y, z, w) = (1.0, 0.5, 0.3, 0.2)
        q1 = Quat(x, y, z, w)
        self.assertEqual(q1[0], x)
        self.assertEqual(q1[1], y)
        self.assertEqual(q1[2], z)
        self.assertEqual(q1[3], w)
        self.assertEqual(q1.x, x)
        self.assertEqual(q1.y, y)
        self.assertEqual(q1.z, z)
        self.assertEqual(q1.w, w)
    def test_axis_angle(self):
        axis = [5, 4, 3]
        angle = 1.2
        q = Quat.AxisAngle(axis, angle)
        self.assertEqual(q.w, math.cos(angle / 2))
    def test_euler_angle(self):
        euler = [0.5, -0.4, 0.3]
        q = Quat.Euler(euler)
        e = q.toEuler()
        np.testing.assert_almost_equal(e[0], euler[0])
        np.testing.assert_almost_equal(e[1], euler[1])
        np.testing.assert_almost_equal(e[2], euler[2])

if __name__ == '__main__':
    unittest.main()
