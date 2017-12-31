#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math

class Quat(object):
    def __init__(self, x, y, z, w):
        self.q = np.array([x, y, z, w])
    def __getattr__(self, name):
	if name == 'x': return self.q[0]
	if name == 'y': return self.q[1]
	if name == 'z': return self.q[2]
	if name == 'w': return self.q[3]
        print name
        return None
    def __getitem__(self, index):
        return self.q[index]
    def __repr__(self):
        return 'x=%g, y=%g, z=%g, w=%g' % (self.q[0], self.q[1], self.q[2], self.q[3])
    def __mul__(self, other):
        (x0, y0, z0, w0) = self.q
        (x1, y1, z1, w1) = other.q
        return Quat(w0*x1 + x0*w1 + y0*z1 - z0*y1,
                    w0*y1 - x0*z1 + y0*w1 + z0*x1,
                    w0*z1 + x0*y1 - y0*x1 + z0*w1,
                    w0*w1 - x0*x1 - y0*y1 - z0*z1)
    def __neg__(self):
        return Quat(-self.q[0], -self.q[1], -self.q[2], -self.q[3])
    def conj(self):
        return Quat(self.q[0], self.q[1], self.q[2], -self.q[3])
    def inv(self):
        return self.conj()
    def toEuler(self):
        (x, y, z, w) = self.q
	# roll (x-axis rotation)
	sinr = 2.0 * (w * x + y * z)
	cosr = 1.0 - 2.0 * (x * x + y * y)
	roll = math.atan2(sinr, cosr)

	# pitch (y-axis rotation)
	sinp = 2.0 * (w * y - z * x)
	if abs(sinp) >= 1:
	    pitch = math.copysign(M_PI / 2, sinp) # use 90 degrees if out of range
	else:
	    pitch = math.asin(sinp);

	# yaw (z-axis rotation)
	siny = 2.0 * (w * z + x * y)
	cosy = 1.0 - 2.0 * (y * y + z * z)
	yaw = math.atan2(siny, cosy)
        return [roll, pitch, yaw]
    @staticmethod
    def AxisAngle(axis, angle):
        ax = np.array(axis, dtype=np.float64)
        ax *= math.sin(0.5 * angle) / np.linalg.norm(ax)
        return Quat(ax[0], ax[1], ax[2], math.cos(0.5 * angle))
    @classmethod
    def Euler(cls, euler):
        qx = Quat.AxisAngle([1, 0, 0], euler[0])
        qy = Quat.AxisAngle([0, 1, 0], euler[1])
        qz = Quat.AxisAngle([0, 0, 1], euler[2])
        return qz * qy * qx
