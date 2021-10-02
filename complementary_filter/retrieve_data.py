#!/usr/bin/env python3

from scipy.io import loadmat
import math
import utils

# load data
GYRO_MATLAB_DATA = loadmat("../data/moving_helicopter_measurements/ArsGyro.mat")
GYRO_WX = GYRO_MATLAB_DATA["wx"][:, 0]
GYRO_WY = GYRO_MATLAB_DATA["wy"][:, 0]
GYRO_WZ = GYRO_MATLAB_DATA["wx"][:, 0]
NUM_GYRO_MEAS = len(GYRO_WX)
DELTA_TIME = 0.01

ACCEL_MATLAB_DATA = loadmat("../data/moving_helicopter_measurements/ArsAccel.mat")
ACCEL_WX = ACCEL_MATLAB_DATA["fx"][:, 0]
ACCEL_WY = ACCEL_MATLAB_DATA["fy"][:, 0]
ACCEL_WZ = ACCEL_MATLAB_DATA["fz"][:, 0]
NUM_ACCEL_MEAS = len(ACCEL_WX)


def get_gyro_measurements(index):
    return utils.AngularVelocities(GYRO_WX[index], GYRO_WY[index], GYRO_WZ[index])


def get_accelerometer_measurements(index):
    return utils.Acceleration(ACCEL_WX[index], ACCEL_WY[index], ACCEL_WZ[index])
