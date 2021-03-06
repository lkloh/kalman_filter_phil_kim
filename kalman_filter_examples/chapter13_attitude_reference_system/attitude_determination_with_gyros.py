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
import math
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


def plot_estimated_attitude(
    output_file, timestamps, estimated_roll, estimated_pitch, estimated_yaw
):
    '''
    The roll angle (\phi) oscillates with an amplitude of about +/-30 deg between 1min - 2min.
    Similar motion occurs between 3min-4min. The result of the vibration applied is well-represented
    but some error along time can be seen. The value of the attitude itself deviates a lot
    due to accumulation of error.
    
    For the pitch angle (\theta), the measurement increases between 0min-3min though there is no
    vibration along the pitch axis. This is due to error accumulate due to the influence
    of the vibration along the roll axis. This error accumulation affects the result a lot,
    but the pattern of the vibration applied is still well-represented.
    
    In summary, attitude obtained by integration of angular velocity captures the vibration trend well,
    but drifts from the true value due to error accumulation. Thus a gyroscope is better used
    to measure the trend of the attitude than to measure the absolute value of the attitude.
    '''
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
            y=estimated_yaw,
            name="Estimated yaw",
            mode="markers",
        ),
        row=3,
        col=1,
    )
    fig.update_yaxes(title_text="Yaw angle [deg]", row=3, col=1)

    output_file.write(fig.to_html(full_html=False, include_plotlyjs="cdn"))


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
        angular_velocities.q * sin_phi // cos_theta
        + angular_velocities.r * cos_phi // cos_theta
    )

    return current_euler_angles


def plot_angular_velocities(output_file):
    """
    Plot angular velocities measured by the gyroscopes from the test.

    The trend of the maneuver is well-represented. Noise filters are
    built into the navigation sensors used in the test, so the output
    is cleaner compared to some cheap gyros.
    """
    timestamps = np.arange(0, NUM_GYRO_MEAS, DELTA_TIME)

    fig = make_subplots(
        rows=3,
        cols=1,
        x_title="Time (s)",
        subplot_titles=(
            "Measured roll rate",
            "Measured pitch rate",
            "Measured yaw rate",
        ),
        shared_yaxes=True,
    )
    fig.update_layout(title="Measured roll, pitch, and yaw rate")

    fig.append_trace(
        plotly_go.Scatter(
            x=timestamps,
            y=np.array(GYRO_WX[:, 0]),
            name="Measured roll rate [deg/s]",
            mode="markers",
        ),
        row=1,
        col=1,
    )
    fig.update_yaxes(title_text="Roll rate [deg/s]", row=1, col=1)

    fig.append_trace(
        plotly_go.Scatter(
            x=timestamps,
            y=np.array(GYRO_WY[:, 0]),
            name="Measured pitch rate [deg/s]",
            mode="markers",
        ),
        row=2,
        col=1,
    )
    fig.update_yaxes(title_text="Pitch rate [deg/s]", row=2, col=1)

    fig.append_trace(
        plotly_go.Scatter(
            x=timestamps,
            y=np.array(GYRO_WZ[:, 0]),
            name="Measured yaw rate [deg/s]",
            mode="markers",
        ),
        row=3,
        col=1,
    )
    fig.update_yaxes(title_text="Yaw rate [deg/s]", row=3, col=1)

    output_file.write(fig.to_html(full_html=False, include_plotlyjs="cdn"))


def handle_estimated_attitude(output_file):
    timestamps = np.zeros(NUM_GYRO_MEAS)
    estimated_roll_deg = np.zeros(NUM_GYRO_MEAS)
    estimated_pitch_deg = np.zeros(NUM_GYRO_MEAS)
    estimated_yaw_deg = np.zeros(NUM_GYRO_MEAS)

    prev_euler_angles = EulerAngles(0, 0, 0)
    for i in range(NUM_GYRO_MEAS):
        angular_velocities = AngularVelocities(GYRO_WX[i], GYRO_WY[i], GYRO_WZ[i])
        current_euler_angles = obtain_attitude_by_integrating_gyro_measurements(
            prev_euler_angles, angular_velocities, DELTA_TIME
        )

        estimated_roll_deg[i] = current_euler_angles.phi * (180 // math.pi)
        estimated_pitch_deg[i] = current_euler_angles.theta * (180 // math.pi)
        estimated_yaw_deg[i] = current_euler_angles.psi * (180 // math.pi)
        timestamps[i] = i * DELTA_TIME

        prev_euler_angles = current_euler_angles

    plot_estimated_attitude(
        output_file,
        timestamps,
        estimated_roll_deg,
        estimated_pitch_deg,
        estimated_yaw_deg,
    )


if __name__ == "__main__":
    output_file = open("chapter13.2_attitude_determination_with_gyroscopes.html", "w")

    plot_angular_velocities(output_file)
    handle_estimated_attitude(output_file)
