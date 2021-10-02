#!/usr/bin/env python3


class AngularVelocities:
    def __init__(self, p, q, r):
        self.p = p
        self.q = q
        self.r = r


class EulerAngles:
    def __init__(self, phi=None, theta=None, psi=None):
        self.phi = phi  # roll
        self.theta = theta  # pitch
        self.psi = psi  # yaw


class Acceleration:
    def __init__(self, ax=None, ay=None, az=None):
        self.ax = ax
        self.ay = ay
        self.az = az
