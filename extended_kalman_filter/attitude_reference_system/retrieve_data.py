#!/usr/bin/env python3

from scipy.io import loadmat
import numpy as np
import math

# load data
GYRO_MATLAB_DATA = loadmat("../../data/moving_helicopter_measurements/ArsGyro.mat")
GYRO_WX = GYRO_MATLAB_DATA["wx"][:, 0]
GYRO_WY = GYRO_MATLAB_DATA["wy"][:, 0]
GYRO_WZ = GYRO_MATLAB_DATA["wx"][:, 0]
NUM_GYRO_MEAS = len(GYRO_WX)
DELTA_TIME = 0.01

ACCEL_MATLAB_DATA = loadmat("../../data/moving_helicopter_measurements/ArsAccel.mat")
ACCEL_WX = ACCEL_MATLAB_DATA["fx"][:, 0]
ACCEL_WY = ACCEL_MATLAB_DATA["fy"][:, 0]
ACCEL_WZ = ACCEL_MATLAB_DATA["fz"][:, 0]
NUM_ACCEL_MEAS = len(ACCEL_WX)

GRAVITATIONAL_ACCEL = 9.8


class AngularVelocities:
    def __init__(self, p, q, r):
        self.p = p
        self.q = q
        self.r = r


def get_gyro_measurements(index):
    return AngularVelocities(GYRO_WX[index], GYRO_WY[index], GYRO_WZ[index])


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


def get_accelerometer_measurements(index):
    return Acceleration(ACCEL_WX[index], ACCEL_WY[index], ACCEL_WZ[index])


def compute_euler_accel(accel):
    theta = math.asin(accel.ax / GRAVITATIONAL_ACCEL)
    phi = math.asin((-1 * accel.ay) / (GRAVITATIONAL_ACCEL * math.cos(theta)))
    return EulerAngles(phi, theta)
