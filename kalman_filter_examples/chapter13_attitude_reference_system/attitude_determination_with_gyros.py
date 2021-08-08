#!/usr/bin/env python3

"""
Chapter 13.2: Attitude determination with gyroscopes
----------------------------------------------------
To obtain the horizontal attitude of a helicopter, we need to know the roll (\phi)
and pitch (\theta) angles of the helicopter.

The gyroscope measures angular velocities (p, q, r) of the helicopter.
In order to obtain the Euler angles from the measurements of the gyroscope,
we need to transform the measurements into rate of change in the Euler
angles and then integrate them. This kinematic relationship is well-known:

[\dot{\phi}]   = [1  \sin(\phi) * \tan(\theta)  \cos(\phi) * \tan(\theta)][p]
[\dot{\theta}] = [0         \cos(\theta)              -\sin(\phi)        ][q]
[\dot{\psi}]   = [0  \sin(\phi) / \cos(\theta)  \cos(\phi) / \cos(\theta)][r]

If the initial values of roll (\phi), pitch (\theta) and yaw (\psi) are known,
current attitude can be obtained by applying the angular velocities measured
by the gyroscope (p, q, r) to the formula above, then integrating w.r.t. time.

However, the measurements from the gyroscope have errors that are accumulated
during integration. Estimationg of quantities with integration is not feasible
for practical applications unless the sensor is very accurate or the time is
very short.

"""
import numpy as np
from numpy.linalg import inv
import math
import plotly.express as px
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
from scipy.io import loadmat

# load data
GYRO_MATLAB_DATA = loadmat("./data/ArsGyro.mat")
GYRO_WX = GYRO_MATLAB_DATA["wx"]
GYRO_WY = GYRO_MATLAB_DATA["wy"]
GYRO_WZ = GYRO_MATLAB_DATA["wx"]
NUM_GYRO_MEAS = len(GYRO_WX)
DELTA_TIME = 0.01


class AngularVelocities:
    def __init__(self, p, q, r):
        self.p = p
        self.q = q
        self.r = r


class EulerAngles:
    def __init__(self, phi, theta, psi):
        self.phi = phi  # roll
        self.theta = theta  # pitch
        self.psi = psi  # yaw


def plot_data(timestamps, estimated_roll, estimated_pitch, estimated_yaw):
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
        title="Estimate roll, pitch, and yaw from a gyroscope using numerical integration"
    )

    fig.append_trace(
        plotly_go.Scatter(
            x=timestamps,
            y=estimated_roll,
            name="Roll",
            mode="markers",
        ),
        row=1,
        col=1,
    )
    fig.update_yaxes(title_text="Roll angle [deg]", row=1, col=1)

    fig.append_trace(
        plotly_go.Scatter(
            x=timestamps,
            y=estimated_pitch,
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
            y=estimated_pitch,
            name="Estimated yaw",
            mode="markers",
        ),
        row=3,
        col=1,
    )
    fig.update_yaxes(title_text="Yaw angle [deg]", row=3, col=1)

    fig.write_html("chapter13.2_attitude_determination_with_a_gyroscope.html")


def obtain_attitude_by_integrating_gyro_measurements(
    prev_euler_angles, angular_velocities, dt
):
    current_euler_angles = EulerAngles(0, 0, 0)

    sin_phi = math.sin(prev_euler_angles.phi)
    cos_phi = math.cos(prev_euler_angles.phi)
    tan_theta = math.tan(prev_euler_angles.theta)
    cos_theta = math.cos(prev_euler_angles.theta)

    current_euler_angles.phi = prev_euler_angles.phi + dt * (
        angular_velocities.p
        + angular_velocities.q * sin_phi * tan_theta
        + angular_velocities.r * cos_phi * tan_theta
    )
    current_euler_angles.theta = prev_euler_angles.theta + dt * (
        angular_velocities.q * cos_phi - angular_velocities.r * sin_phi
    )
    current_euler_angles.psi = prev_euler_angles.psi + dt * (
        angular_velocities.q * sin_phi / cos_theta
        + angular_velocities.r * cos_phi / cos_theta
    )

    return current_euler_angles


def kalman_filter(z, prev_x_estimate, prev_P_estimate):
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


if __name__ == "__main__":
    prev_euler_angles = EulerAngles(0, 0, 0)

    timestamps = np.zeros(NUM_GYRO_MEAS)
    estimated_roll = np.zeros(NUM_GYRO_MEAS)
    estimated_pitch = np.zeros(NUM_GYRO_MEAS)
    estimated_yaw = np.zeros(NUM_GYRO_MEAS)

    current_euler_angles = EulerAngles(0, 0, 0)
    current_euler_angles.psi = 1

    for i in range(NUM_GYRO_MEAS):
        angular_velocities = AngularVelocities(GYRO_WX[i], GYRO_WY[i], GYRO_WZ[i])
        current_euler_angles = obtain_attitude_by_integrating_gyro_measurements(
            prev_euler_angles, angular_velocities, DELTA_TIME
        )

        estimated_roll[i] = current_euler_angles.phi
        estimated_pitch[i] = current_euler_angles.theta
        estimated_yaw[i] = current_euler_angles.psi
        timestamps[i] = i * DELTA_TIME

        prev_euler_angles = current_euler_angles

    plot_data(timestamps, estimated_roll, estimated_pitch, estimated_yaw)
