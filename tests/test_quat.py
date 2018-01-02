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
        q = Quat.axisAngle(axis, angle)
        self.assertEqual(q.w, math.cos(angle / 2))
    def test_euler_angle(self):
        ang = [0.5, -0.4, 0.3]
        q = Quat.euler(ang)
        e = q.toEuler()
        np.testing.assert_almost_equal(e[0], ang[0])
        np.testing.assert_almost_equal(e[1], ang[1])
        np.testing.assert_almost_equal(e[2], ang[2])
    def test_rot(self):
        axis = [1, 0, 0]
        angle = 0.5 * math.pi
        q = Quat.axisAngle(axis, angle)
        v = np.array([0, 1, 0])
        vq = q.rotate(v)
        np.testing.assert_almost_equal(vq[0], 0)
        np.testing.assert_almost_equal(vq[1], 0)
        np.testing.assert_almost_equal(vq[2], 1)
        r = q.toRot()
        vr = r.dot(v)
        np.testing.assert_almost_equal(vq, vr)
    def test_explog(self):
        axis = np.array([1, 0, 0])
        angle = 0.5 * math.pi
        q = Quat.axisAngle(axis, angle)
        w = q.log()
        qw = Quat.exp(w)
        np.testing.assert_almost_equal(q.q, qw.q)
        np.testing.assert_almost_equal(axis * angle, w)

if __name__ == '__main__':
    unittest.main()
