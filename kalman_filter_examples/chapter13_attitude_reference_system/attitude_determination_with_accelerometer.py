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
    """
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
    """
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
    plot_measured_acceleration()
    #handle_estimated_attitude()
