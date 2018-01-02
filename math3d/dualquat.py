#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
import random
from quat import Quat

class ScrewParameter(object):
    def __init__(self, theta, d, l, m):
        self.theta = theta
        self.d = d
        self.l = l
        self.m = m

class DualQuat(object):
    def __init__(self, p, q):
        self.p = p              # ordinal
        self.q = q              # dual
    def transformVec(self, v):
        vq = DualQuat(Quat.identity(), Quat(v[0], v[1], v[2], 0.0))
        wq = self * vq * self.conj3()
        return [wq.q[0], wq.q[1], wq.q[2]]
    def transformScrew(self, w):
        return self * w * self.conj2()
    def translation(self):
        t =  2 * self.q * self.p.inv()
        return t.q[0:3]
    def __repr__(self):
        return 'ord=(%s), dual=(%s)' % (str(self.p), str(self.q))
    def __mul__(self, other):
        return DualQuat(self.p * other.p, self.p * other.q + self.q * other.p)
    def conj1(self):
        return DualQuat(self.p, -self.q)
    def conj2(self):
        return DualQuat(self.p.conj(), self.q.conj())
    def conj3(self):
        return DualQuat(self.p.conj(), -self.q.conj())
    def screwParameter(self):
        theta = 2.0 * math.acos(self.p.w)
        n = np.linalg.norm(self.p.q[0:3])
        d = -2.0 * self.q.w / n
        l = self.p.q[0:3] / n
        m = (self.q.q[0:3] - 0.5 * d * self.p.w * l) / n
        return ScrewParameter(theta, d, l, m)
    @staticmethod
    def trans(r, t):
        return DualQuat(r, 0.5 * t * r)
    @staticmethod
    def fromTrans(tr):
        t = Quat(tr.pos[0], tr.pos[1], tr.pos[2], 0.0)
        return DualQuat.trans(tr.q, t)
    @staticmethod
    def random():
        x = random.uniform(-1.0, 1.0)
        y = random.uniform(-1.0, 1.0)
        z = random.uniform(-1.0, 1.0)
        return DualQuat.trans(Quat.random(), Quat(x, y, z, 0.0))
