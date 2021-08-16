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

[\dot{\phi}  ] = [1  \sin(\phi) * \tan(\theta)  \cos(\phi) * \tan(\theta)][p]
[\dot{\theta}] = [0         \cos(\theta)              -\sin(\phi)        ][q]
[\dot{\psi}  ] = [0  \sin(\phi) / \cos(\theta)  \cos(\phi) / \cos(\theta)][r]

However, there is a problem with using this equation in a Kalman filter's system model.
It should be written like this:
                             [ \dot{\phi} ]   [   ][ \phi ]
x_{k+1} = A * x_K + w_k <==> [\dot{\theta}] = [ ? ][\theta] + w
                             [ \dot{\psi} ]   [   ][ \psi ]
We cannot extract Euler angles from the matrix, and therefore cannot use the
Kalman filter in this form. We need to find a way to change the shape into a form
that can be used in Kalman filter. One way to do this is select a different
set of state variables.

Instead of Euler angles, we use the quaternion as the state variable. It is another
way to express attitude, and the derivation is well known.
    [Q1]
x = [Q2]
    [Q3]
    [Q4]

The relationship between angular velocity and rate of change in quaternion is also
well-known:

[\dot{Q1}]         [0 -p -q -r][Q1]
[\dot{Q2}] = 1/2 * [p 0  r  -q][Q2]
[\dot{Q3}]         [q -r 0  p ][Q3]
[\dot{Q4}]         [r q  -p 0 ][Q4]

Only the state variable has been changed -- and now the Euler angles can be extracted
from the expression. The system model becomes:

[Q1_{k+1}]   (                      [0 -p -q -r])   [Q1]
[Q2_{k+1}] = (I + \delta(t) * 1/2 * [p 0  r  -q]) * [Q2]
[Q3_{k+1}]   (                      [q -r 0  p ])   [Q3]
[Q4_{k+1}]   (                      [r q  -p 0 ])   [Q4]
and matrix A of the system model is
                      [0  -p -q -r]
I + \delta(t) * 1/2 * [p  0  r  -q]
                      [q  -r 0  p ]
                      [r  q  -p 0 ]
We see that elements of A now change according to angular velocity.

The Euler angles calculated from the accelerometer will be used as measurements
to the Kalman filter. Since the state variable is the quaternion, the acceleration
measurements must be transform to the quaternion. The transformation expression is:
[Q1]   [cos(phi/2) * cos(theta/2) * cos(psi/2) + sin(phi/2) * sin(theta/2) * sin(psi/2)]
[Q2] = [sin(phi/2) * cos(theta/2) * cos(psi/2) - cos(phi/2) * sin(theta/2) * sin(psi/2)]
[Q3]   [cos(phi/2) * sin(theta/2) * cos(psi/2) + sin(phi/2) * cos(theta/2) * sin(psi/2)]
[Q4]   [cos(phi/2) * cos(theta/2) * sin(psi/2) - sin(phi/2) * sin(theta/2) * cos(psi/2)]

By applying attitude from accelerometer to the above expression, the quaternion
measurement is obtained. This is the measurement z_k for the Kalman filter.

All state variables of the quaternion are measured, so the state-to-measurement
matrix H simply is the 4x4 identify matrix.

The resulting fusion of accelerometer and gyroscope shows the trajectory of roll and pitch
angles well, and the oscillation of +/- 30 deg is accurately shown.
But it is rare that a system model can be expression in a linear form as the
Kalman filter requires. A generalized approach is needed in most cases.
"""
import numpy as np
from numpy.linalg import inv
import math
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
from scipy.io import loadmat

# load gyroscope measurements
GYRO_MATLAB_DATA = loadmat("./data/ArsGyro.mat")
GYRO_WX = GYRO_MATLAB_DATA["wx"][:, 0]
GYRO_WY = GYRO_MATLAB_DATA["wy"][:, 0]
GYRO_WZ = GYRO_MATLAB_DATA["wx"][:, 0]
NUM_GYRO_MEAS = len(GYRO_WX)

ACCEL_MATLAB_DATA = loadmat("./data/ArsAccel.mat")
ACCEL_WX = ACCEL_MATLAB_DATA["fx"][:, 0]
ACCEL_WY = ACCEL_MATLAB_DATA["fy"][:, 0]
ACCEL_WZ = ACCEL_MATLAB_DATA["fz"][:, 0]
NUM_ACCEL_MEAS = len(ACCEL_WX)

DELTA_TIME = 0.01

GRAVITATIONAL_ACCEL = 9.8

"""
Noise covariance matrices are related to characteristics of the system signal.
Hard for theoretically analyze what they should be - better to analyze real-time data.
These are design factors, values decided after monitoring the variance of performance
through trial an error. These are example values.
"""
# Covariance of state transition noise
Q = np.matrix(
    [
        [1e-4, 0, 0, 0],
        [0, 1e-4, 0, 0],
        [0, 0, 1e-4, 0],
        [0, 0, 0, 1e-4],
    ]
)
# Covariance of measurement noise
R = np.matrix(
    [
        [10, 0, 0, 0],
        [0, 10, 0, 0],
        [0, 0, 10, 0],
        [0, 0, 0, 10],
    ]
)

# State-to-measurement matrix
H = np.identity(4)

## Data structions
class AngularVelocities:
    def __init__(self, p=None, q=None, r=None):
        self.p = p
        self.q = q
        self.r = r


class EulerAngles:
    def __init__(self, phi=None, theta=None, psi=None):
        self.phi = phi  # roll
        self.theta = theta  # pitch
        self.psi = psi  # yaw


def plot_estimates(timestamps, estimated_phi, estimated_theta, estimated_psi):
    fig = make_subplots(
        rows=3,
        cols=1,
        x_title="Time (s)",
        subplot_titles=(
            "Estimated roll",
            "Estimated pitch",
            "Estimated yaw",
        ),
        shared_yaxes=True,
    )
    fig.update_layout(
        title="Estimate roll, pitch, and yaw from Kalman filter combining gyroscope and accelerometer"
    )

    fig.append_trace(
        plotly_go.Scatter(
            x=timestamps,
            y=estimated_phi,
            name="Estimated roll",
            mode="markers",
        ),
        row=1,
        col=1,
    )
    fig.update_yaxes(title_text="Roll angle [deg]", row=1, col=1)

    fig.append_trace(
        plotly_go.Scatter(
            x=timestamps,
            y=estimated_theta,
            name="Estimated pitch",
            mode="markers",
        ),
        row=2,
        col=1,
    )
    fig.update_yaxes(title_text="Pitch angle [deg]", row=2, col=1)

    fig.append_trace(
        plotly_go.Scatter(
            x=timestamps,
            y=estimated_psi,
            name="Estimated yaw",
            mode="markers",
        ),
        row=3,
        col=1,
    )
    fig.update_yaxes(title_text="Yaw angle [deg]", row=3, col=1)

    fig.write_html("chapter13.4_estimating_attitude_with_sensorfusion.html")


def kalman_filter(prev_x_estimate, prev_P_estimate, A):
    # Step I: Predict state
    x_predict = A * prev_x_estimate
    # Step I: Predict error covariance
    P_predict = A * prev_P_estimate * A.transpose() + Q

    # Step II: Compute Kalman Gain
    K = P_predict * H.transpose() * inv(H * P_predict * H.transpose() + R)

    # Step III: Compute estimate
    x_estimate = x_predict + K * (z - H * x_predict)

    # Step IV: Compute error covariance
    P_estimate = P_predict - K * H * P_predict

    return [x_estimate, P_estimate]


def acceleration_to_euler_angles(ax, ay):
    theta = math.asin(ax / GRAVITATIONAL_ACCEL)
    phi = math.asin((-1 * ay) / (GRAVITATIONAL_ACCEL * math.cos(theta)))
    return EulerAngles(phi, theta, 0)


def euler_angles_to_quaternion(euler_angles):
    sin_phi = math.sin(euler_angles.phi // 2)
    sin_theta = math.sin(euler_angles.theta // 2)
    sin_psi = math.sin(euler_angles.psi // 2)

    cos_phi = math.cos(euler_angles.phi // 2)
    cos_theta = math.cos(euler_angles.theta // 2)
    cos_psi = math.cos(euler_angles.psi // 2)

    return np.matrix(
        [
            [cos_phi * cos_theta * cos_psi + sin_phi * sin_theta * sin_psi],
            [sin_phi * cos_theta * cos_psi - cos_phi * sin_theta * sin_psi],
            [cos_phi * sin_theta * cos_psi + sin_phi * cos_theta * sin_psi],
            [cos_phi * cos_theta * sin_psi - sin_phi * sin_theta * cos_psi],
        ]
    )


def quaternion_to_euler_angles(Q):
    phi = np.arctan2(
        2 * (Q[2] * Q[3] + Q[0] * Q[1]), 1 - 2 * (Q[1] * Q[1] + Q[2] * Q[2])
    )
    theta = -1 * np.arcsin(2 * (Q[1] * Q[3] - Q[0] * Q[2]))
    psi = np.arctan2(
        2 * (Q[1] * Q[2] + Q[0] * Q[3]), 1 - 2 * (Q[2] * Q[2] + Q[3] * Q[3])
    )
    return EulerAngles(phi, theta, psi)


if __name__ == "__main__":
    if NUM_GYRO_MEAS != NUM_ACCEL_MEAS:
        print(
            "Error, number of gyroscope measurements (%i) is different from number of accelerometer measurements(%i)",
            NUM_GYRO_MEAS,
            NUM_ACCEL_MEAS,
        )

    estimated_phi = np.zeros(NUM_GYRO_MEAS)
    estimated_theta = np.zeros(NUM_GYRO_MEAS)
    estimated_psi = np.zeros(NUM_GYRO_MEAS)
    timestamps = np.zeros(NUM_GYRO_MEAS)

    # All Euler angles are 0 at the start
    prev_x_estimate = np.matrix([[1], [0], [0], [0]])
    prev_P_estimate = np.identity(4)

    for measurement_idx in range(NUM_GYRO_MEAS):
        # Define system transition matrix
        p = GYRO_WX[measurement_idx]
        q = GYRO_WY[measurement_idx]
        r = GYRO_WZ[measurement_idx]
        A = np.identity(4) + DELTA_TIME * 0.5 * np.matrix(
            [
                [0, -1 * p, -1 * q, -1 * r],
                [p, 0, r, -1 * q],
                [q, -1 * r, 0, p],
                [r, q, -1 * p, 0],
            ]
        )

        # get measurements
        ax = ACCEL_WX[measurement_idx]
        ay = ACCEL_WY[measurement_idx]

        euler_angles_from_accel = acceleration_to_euler_angles(ax, ay)
        z = euler_angles_to_quaternion(euler_angles_from_accel)

        [current_x_estimate, current_P_estimate] = kalman_filter(
            prev_x_estimate, prev_P_estimate, A
        )
        estimated_euler_angles = quaternion_to_euler_angles(current_x_estimate)

        timestamps[measurement_idx] = measurement_idx * DELTA_TIME
        estimated_phi[measurement_idx] = estimated_euler_angles.phi * (180 // math.pi)
        estimated_theta[measurement_idx] = estimated_euler_angles.theta * (
            180 // math.pi
        )
        estimated_psi[measurement_idx] = estimated_euler_angles.psi * (180 // math.pi)

    plot_estimates(timestamps, estimated_phi, estimated_theta, estimated_psi)
