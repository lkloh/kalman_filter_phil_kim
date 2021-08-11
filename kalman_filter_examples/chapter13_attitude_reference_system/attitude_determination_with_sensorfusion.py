#!/usr/bin/env python3

"""
Chapter 13.4: Attitude determination with sensorfusion
------------------------------------------------------
Finding attitude using measurements from a gyroscope has shortcomings --
it is sensitive to changes in attitude, but the error diverges along time
due to its accumulation. Thus it is a good sensor for a short amount of time,
but a bad one for long periods of time.

Finding attitude using measurements from an accelerometer also has shortcomings --
while the error stays within a certain range and does not grow with time,
the error in attitude is large. Thus it is a good sensor for a long period of time,
but a bad one for short periods.

We could obtain accurate estimates of attitude with a more expensive sensor,
but that is not always possible for economic reasons. Instead, we could fix the
problem of error accumulation with the accelerometer, using a technique known as
sensor fusion.

Sensor fusion combines the outputs of various sensors to by combining them.
It is important that the sensors chosen complement each other --
so the shortcoming of one sensor is covered by the advantage of the other.

To fuse the gyroscope and accelerometer, we do the following:

                                                -----------
Accelerometer -->   Attitude     measurement   | Gyroscope |  attitude
                  determination ------------->  -----------  ---------->
                                               Kalman Filter

The attitude (with large bounded error) determined by the accelerometer
becomes the measurement in the Kalman Filter. The error in the gyroscope
is corrected by the accelerometer as the attitude. The error from the gyroscope
diverges and cannot otherwise be used to calibrate data.

System Model for Sensor Fusion
------------------------------
The Kalman filter algorithm itself does not change between different applications.
The system model does vary -- the most important thing when using Kalman filter
is to find an appropriate and accurate system model.

The physical quantity we want to estimate is the horizontal attitude -- so we set the
attitude as the state variable.
    [ \phi ] roll
x = [\theta] pitch
    [ \psi ] yaw - any value can be assigned since we're not interested in vertical attitude

The relationship between angular velocities (p, q, r) from the gyroscope and
rate of change (\dot{\phi}, \dot{\theta}, \dot{\psi}) in the Euler angles is well-known:

[\dot{\phi}]   = [1  \sin(\phi) * \tan(\theta)  \cos(\phi) * \tan(\theta)][p]
[\dot{\theta}] = [0         \cos(\theta)              -\sin(\phi)        ][q]
[\dot{\psi}]   = [0  \sin(\phi) / \cos(\theta)  \cos(\phi) / \cos(\theta)][r]
"""
import numpy as np
import math
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
from scipy.io import loadmat
