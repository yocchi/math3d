#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
from quat import Quat

class DualQuat(object):
    def __init__(self, p, q):
        self.p = p              # ordinal
        self.q = q              # dual
    def transformVec(self, v):
        vq = DualQuat(Quat.Identity(), Quat(v[0], v[1], v[2], 0.0))
        wq = self * vq * self.conj3()
        return [wq.q[0], wq.q[1], wq.q[2]]
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

    @staticmethod
    def Trans(r, t):
        return DualQuat(r, 0.5 * t * r)

