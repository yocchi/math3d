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
        x0, y0, z0, w0 = self.q
        x1, y1, z1, w1 = other.q
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
    def rotate(self, v):
        x, y, z, w = self.q
        x0 = v[0] * w + v[2] * y - v[1] * z
	x1 = v[1] * w + v[0] * z - v[2] * x
	x2 = v[2] * w + v[1] * x - v[0] * y
	x3 = v[0] * x + v[1] * y + v[2] * z
        return [w * x0 + x * x3 + y * x2 - z * x1,
                w * x1 + y * x3 + z * x0 - x * x2,
                w * x2 + z * x3 + x * x1 - y * x0]
    def toEuler(self):
        x, y, z, w = self.q
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
    def toRot(self):
        x, y, z, w = self.q
	wx, wy, wz = (2.0*w*x, 2.0*w*y, 2.0*w*z)
	xx, yy, zz = (2.0*x*x, 2.0*y*y, 2.0*z*z)
	xy, yz, xz = (2.0*x*y, 2.0*y*z, 2.0*x*z)
        rot = np.zeros((3, 3))
        rot[0, 0] = 1 - yy - zz
        rot[0, 1] = xy - wz
        rot[0, 2] = xz + wy
        rot[1, 0] = xy + wz
	rot[1, 1] = 1 - xx - zz
	rot[1, 2] = yz - wx
        rot[2, 0] = xz - wy
	rot[2, 1] = yz + wx
	rot[2, 2] = 1 - xx - yy
        return rot
    @staticmethod
    def Identity():
        return Quat(1.0, 0.0, 0.0, 0.0)
    @staticmethod
    def AxisAngle(axis, angle):
        ax = np.array(axis, dtype=np.float64)
        ax *= math.sin(0.5 * angle) / np.linalg.norm(ax)
        return Quat(ax[0], ax[1], ax[2], math.cos(0.5 * angle))
    @staticmethod
    def Euler(euler):
        qx = Quat.AxisAngle([1, 0, 0], euler[0])
        qy = Quat.AxisAngle([0, 1, 0], euler[1])
        qz = Quat.AxisAngle([0, 0, 1], euler[2])
        return qz * qy * qx
