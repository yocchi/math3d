#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
import random
from quat import Quat

class Trans(object):
    def __init__(self, v, q):
        self.pos = np.array(v)  # position
        self.q = q    # rotation
    def transformVec(self, v):
        return self.pos + self.q.rotate(v)
    def inv(self):
        iq = self.q.inv()
        return Trans(iq.rotate([-self.pos[0], -self.pos[1], -self.pos[2]]), iq)
    def __repr__(self):
        return 'pos=(%s), quat=(%s)' % (str(self.pos), str(self.q))
    def __mul__(self, other):
        return Trans(self.pos + self.q.rotate(other.pos),
                     self.q * other.q)
    @staticmethod
    def random():
        x = random.uniform(-1.0, 1.0)
        y = random.uniform(-1.0, 1.0)
        z = random.uniform(-1.0, 1.0)
        return Trans(np.array([x, y, z]), Quat.random())
