#!/usr/bin/env python3

"""
Chapter 13.3: Attitude determination with accelerometers
--------------------------------------------------------
When using an accelerometer to measure the accelerations (ax, ay, az),
there are various accelerations mixed in, such as:
* gravitational acceleration
* accelerations caused by change of velocity in magnitude and direction

The equation for this expression is:
[ax]   [\dot{vx}]   [0    vz   -vy][p]       [       \sin(\theta)       ]
[ay] = [\dot{vy}] + [-vz  0    vx ][q] + g * [-\cos(\theta) * \sin(\phi)]
[az]   [\dot{vz}]   [vy   -vx  0  ][r]       [-\cos(\theta) * \cos(\phi)]
where vx, vy, vz are velocities along each body frame,
and p, q, r are angular velocities along each axis in the body frame,
and g is the gravitational acceleration.

If the values of all the rest of the terms are know, then horizontal
attitude could be obtained.
* Accelerations (ax, ay, az) is obtained from the accelerometer's measurements
* Angular velocities (p, q, r) are obtained from the gyroscope's measurements
* Gravitation acceleration (g) is known
* Don't know what the velocities along the body frame (vx, vy, vz) are
  - has to be measured by an very expensive set of sensors
* Don't know the angular velocities (p, q, r) along the axes in the body frame
  are - has to be measured by an very expensive set of sensors
There is no way to determine the attitude with an ordinary navigation sensor.

But determining the attitude possible if the system is stationary or moving
with constant velocity. This is a reasonable assumption in some cases:
* Helicopter missions usually are done while hovering or at constant velocity
* Walking robots are usually stationary or move at constant velocity.

When the system is stationary:
* Velocities and acceleration along each axis in the body frame is all 0.
* Since there is no change in attitude, angular velocity is also 0
* Thus (vx = vy = vz = 0), (\dot{vx} = \dot{vy} = \dot{vz} = 0)
  and (p = q = r = 0)

When the system moves with constant velocity:
* Acceleration along each axis in the body frame is 0
* Since there is no change in attitude, angular velocity is also 0
* Thus (\dot{vx} = \dot{vy} = \dot{vz} = 0) and (p = q = r = 0)

Therefore the kinematic equation simplifies to:
[ax]       [       \sin(\theta)       ]
[ay] = g * [-\cos(\theta) * \sin(\phi)]
[az]       [-\cos(\theta) * \cos(\phi)]

The formulae for roll and pitch angles can now be derived. These are just
approximations for the horizontal attitude when the velocity is slow enough
and the rate of change in magnitude and direction of the velocity is small.
                             -ay
* \phi (roll) = sin^{-1} ---------------
                         g * \cos(theta)
                             ax
* \theta (pitch) = sin^{-1} ---
                             g
"""
import numpy as np
import math
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
from scipy.io import loadmat

# load data
ACCEL_MATLAB_DATA = loadmat("./data/ArsAccel.mat")
ACCEL_WX = ACCEL_MATLAB_DATA["fx"][:, 0]
ACCEL_WY = ACCEL_MATLAB_DATA["fy"][:, 0]
ACCEL_WZ = ACCEL_MATLAB_DATA["fz"][:, 0]
NUM_ACCEL_MEAS = len(ACCEL_WX)
DELTA_TIME = 0.01

GRAVITATIONAL_ACCEL = 9.8


class EulerAngles:
    def __init__(self, phi, theta, psi):
        self.phi = phi  # roll
        self.theta = theta  # pitch
        self.psi = psi  # yaw


def plot_estimated_attitude(timestamps, estimated_roll, estimated_pitch):
    fig = make_subplots(
        rows=2,
        cols=1,
        x_title="Time (s)",
        subplot_titles=(
            "Estimated roll",
            "Estimated pitch",
        ),
        shared_yaxes=True,
    )
    fig.update_layout(
        title="Estimate roll and pitch obtained from measurements from an accelerometer"
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

    fig.write_html("chapter13.3_attitude_determination_with_accelerometer.html")


def obtain_euler_angles_from_acceleration_measurements(ax, ay):
    theta = math.asin(ax / GRAVITATIONAL_ACCEL)
    phi = math.asin((-1 * ay) / (GRAVITATIONAL_ACCEL * math.cos(theta)))
    return EulerAngles(theta, phi, None)


def plot_measured_acceleration():
    """
    Plot measured accelerations with an accelerometer.

    Acceleration along the pitch axis is measured during the roll maneuver -
    roll maneuver is a periodic oscillation about the roll axis, which causes
    the pitch axis to move up and down w.r.t the horizontal plane.
    When this happens, gravitational acceleration, centrifugal acceleration,
    and others are measured at the accelerometer mounted along the pitch axis.

    The same happens for the pitch maneuver - it causes acceleration to be
    measured along the roll-axis.
    """
    timestamps = np.arange(0, NUM_ACCEL_MEAS, DELTA_TIME)

    fig = make_subplots(
        rows=3,
        cols=1,
        x_title="Time (s)",
        subplot_titles=(
            "Measured acceleration along x-axis",
            "Measured acceleration along y-axis",
            "Measured acceleration along z-axis",
        ),
        shared_yaxes=True,
    )
    fig.update_layout(title="Measured acceleration by an accelerometer")

    fig.append_trace(
        plotly_go.Scatter(
            x=timestamps,
            y=np.array(ACCEL_WX),
            name="Measured accelerometer along x-axis [m/s^2]",
            mode="markers",
        ),
        row=1,
        col=1,
    )
    fig.update_yaxes(title_text="Acceleration along x-axis [m/s^2]", row=1, col=1)

    fig.append_trace(
        plotly_go.Scatter(
            x=timestamps,
            y=np.array(ACCEL_WY),
            name="Measured accelerometer along y-axis [m/s^2]",
            mode="markers",
        ),
        row=2,
        col=1,
    )
    fig.update_yaxes(title_text="Acceleration along y-axis [m/s^2]", row=2, col=1)

    fig.append_trace(
        plotly_go.Scatter(
            x=timestamps,
            y=np.array(ACCEL_WZ),
            name="Measured accelerometer along z-axis [m/s^2]",
            mode="markers",
        ),
        row=3,
        col=1,
    )
    fig.update_yaxes(title_text="Acceleration along z-axis [m/s^2]", row=3, col=1)

    fig.write_html("chapter13.3_measured_acceleration.html")


def handle_estimated_attitude():
    timestamps = np.zeros(NUM_ACCEL_MEAS)
    estimated_roll_deg = np.zeros(NUM_ACCEL_MEAS)
    estimated_pitch_deg = np.zeros(NUM_ACCEL_MEAS)

    for i in range(NUM_ACCEL_MEAS):
        euler_angles = obtain_euler_angles_from_acceleration_measurements(
            ACCEL_WX[i], ACCEL_WY[i]
        )

        estimated_roll_deg[i] = euler_angles.phi * (180 // math.pi)
        estimated_pitch_deg[i] = euler_angles.theta * (180 // math.pi)
        timestamps[i] = i * DELTA_TIME

    plot_estimated_attitude(
        timestamps,
        estimated_roll_deg,
        estimated_pitch_deg,
    )


if __name__ == "__main__":
    plot_measured_acceleration()
    handle_estimated_attitude()
