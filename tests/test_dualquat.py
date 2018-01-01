#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from math3d import Quat, DualQuat, Trans, crossMat
import unittest
import math
import numpy as np

np.set_printoptions(precision=3, suppress=True)

class TestDualQuat(unittest.TestCase):
    def test_trans(self):
        t = [1.0, 2.0, 3.0]
        q = DualQuat.Trans(t=Quat(t[0], t[1], t[2], 0.0),
                           r=Quat.Euler([0.5 * math.pi, 0.0, 0.0]))
        v = [1.0, 0.0, 2.0]
        vq = q.transformVec(v)
        vans = [2, 0, 3]
        np.testing.assert_almost_equal(vq, vans)
        tq = q.translation()
        np.testing.assert_almost_equal(t, tq)

    def test_handeye(self):
        'Hand-Eye Calibration Using Dual Quaternions'
        X = Trans([0.1, 0.2, 0.3],
                  Quat.Euler([0.4, -0.5, 0.6]))
        Z = Trans([0.9, 0.7, 0.8],
                  Quat.Euler([0.15, -0.25, 0.35]))
        N = 3
        M = N - 1
        B = [Trans.Random() for i in range(N)]
        A = [X * B[i].inv() * Z for i in range(N)]
        a = [DualQuat.fromTrans(A[i+1] * A[i].inv()) for i in range(M)]
        b = [DualQuat.fromTrans(B[i+1].inv() * B[i]) for i in range(M)]
        AX = A[1] * A[0].inv() * X
        XB = X * B[1].inv() * B[0]
        np.testing.assert_almost_equal(AX.pos, XB.pos)
        np.testing.assert_almost_equal(AX.q.q, XB.q.q)
        S = np.zeros((6*M, 8))
        for i in range(M):
            np.testing.assert_almost_equal(a[i].p.w, b[i].p.w)
            np.testing.assert_almost_equal(a[i].q.w, b[i].q.w)
            S[i*6+0:i*6+3, 0:3] = crossMat(a[i].p.q[0:3] + b[i].p.q[0:3])
            S[i*6+0:i*6+3, 3] = a[i].p.q[0:3] - b[i].p.q[0:3]
            S[i*6+3:i*6+6, 0:3] = crossMat(a[i].q.q[0:3] + b[i].q.q[0:3])
            S[i*6+3:i*6+6, 3] = a[i].q.q[0:3] - b[i].q.q[0:3]
            S[i*6+3:i*6+6, 4:8] = S[i*6+0:i*6+3, 0:4]
        U, s, V = np.linalg.svd(S, full_matrices=True, compute_uv=True)
        np.testing.assert_almost_equal(s[6], 0.0)
        np.testing.assert_almost_equal(s[7], 0.0)
        v7 = V[6, 0:8].T
        v8 = V[7, 0:8].T
        np.testing.assert_almost_equal(np.linalg.norm(v7), 1.0)
        np.testing.assert_almost_equal(np.linalg.norm(v8), 1.0)
        np.testing.assert_almost_equal(S.dot(v7), np.zeros(S.shape[0]))
        np.testing.assert_almost_equal(S.dot(v8), np.zeros(S.shape[0]))
        u1 = v7[0:4]
        v1 = v7[4:8]
        u2 = v8[0:4]
        v2 = v8[4:8]
        coef = [u1.dot(v1), u1.dot(v2) + u2.dot(v1), u2.dot(v2)]
        s1, s2 = np.roots(coef)
        coef = [u1.dot(u1), 2.0 * u1.dot(u2), u2.dot(u2)]
        v1 = np.polyval(coef, s1)
        v2 = np.polyval(coef, s2)
        s, v = (s1, v1) if v1 > v2 else (s2, v2)
        l2 = math.sqrt(1.0 / v)
        l1 = s * l2
        x = l1 * v7 + l2 * v8
        x = DualQuat(Quat(x[0], x[1], x[2], x[3]),
                     Quat(x[4], x[5], x[6], x[7]))
        np.testing.assert_almost_equal(x.p.norm(), 1.0)
        np.testing.assert_almost_equal(x.p.dot(x.q), 0.0)
        np.testing.assert_almost_equal(X.q.toEuler(), x.p.toEuler())
        np.testing.assert_almost_equal(X.pos, x.translation())

if __name__ == '__main__':
    unittest.main()
