#!/usr/bin/env python3

import numpy as np
from get_radar import generate_radar_measurement
from ukf import RadarUKF
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go

Q = 0.01 * np.eye(3)  # Covariance of of state transition noise
R = 100  # Covariance of measurement noise
KAPPA = 0

N_STATE_VARS = 3  # state vars are: x-position,
N_MEASUREMENT_VARS = 1
DELTA_TIME = 0.05
N_RADAR_SAMPLES = 50


def plot_estimates(
    saved_timestamps, saved_position, saved_velocity, saved_altitude, saved_measurements
):
    fig = make_subplots(
        rows=4,
        cols=1,
        x_title="Time (s)",
    )
    fig.update_layout(title="Estimated radar")

    fig.append_trace(
        plotly_go.Scatter(
            x=saved_timestamps,
            y=saved_measurements,
            name="Slant measurements [m]",
            mode="lines+markers",
        ),
        row=1,
        col=1,
    )

    fig.append_trace(
        plotly_go.Scatter(
            x=saved_timestamps,
            y=saved_position,
            name="Estimated x-position [m]",
            mode="lines+markers",
        ),
        row=2,
        col=1,
    )

    fig.append_trace(
        plotly_go.Scatter(
            x=saved_timestamps,
            y=saved_velocity,
            name="Estimated velocity [m/s]",
            mode="lines+markers",
        ),
        row=3,
        col=1,
    )

    fig.append_trace(
        plotly_go.Scatter(
            x=saved_timestamps,
            y=saved_altitude,
            name="Estimated altitude [m]",
            mode="lines+markers",
        ),
        row=4,
        col=1,
    )

    fig.write_html("chapter15.4_radar_tracking.html")


def run_ukf():
    saved_timestamps = np.zeros(N_RADAR_SAMPLES)
    saved_position = np.zeros(N_RADAR_SAMPLES)
    saved_velocity = np.zeros(N_RADAR_SAMPLES)
    saved_altitude = np.zeros(N_RADAR_SAMPLES)
    saved_measurements = np.zeros(N_RADAR_SAMPLES)

    prev_state = np.array([0, 90, 1100])
    prev_error_covariance = 100 * np.eye(3)

    radar_ukf = RadarUKF(Q, R, KAPPA, DELTA_TIME, N_STATE_VARS, N_MEASUREMENT_VARS)

    for i in range(N_RADAR_SAMPLES):
        radar_meas = generate_radar_measurement(prev_state, DELTA_TIME)
        [current_state, current_error_covariance] = radar_ukf.run_ukf(
            radar_meas, prev_state, prev_error_covariance
        )

        saved_timestamps[i] = i * DELTA_TIME
        saved_measurements[i] = radar_meas
        saved_position[i] = current_state[0]
        saved_velocity[i] = current_state[1]
        saved_altitude[i] = current_state[2]

        prev_state = current_state
        prev_error_covariance = current_error_covariance

    return [
        saved_timestamps,
        saved_position,
        saved_velocity,
        saved_altitude,
        saved_measurements,
    ]


if __name__ == "__main__":
    [
        saved_timestamps,
        saved_position,
        saved_velocity,
        saved_altitude,
        saved_measurements,
    ] = run_ukf()

    plot_estimates(
        saved_timestamps,
        saved_position,
        saved_velocity,
        saved_altitude,
        saved_measurements,
    )
