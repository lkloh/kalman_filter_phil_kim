#!/usr/bin/env python3

import numpy as np
import math
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
from get_radar import generate_radar_measurement
from radar_ekf import RadarEKF

NUM_SAMPLES = 100
DELTA_TIME = 0.5
A = np.identity(3) + DELTA_TIME * np.matrix(
    [
        [0, 1, 0],
        [0, 0, 0],
        [0, 0, 0],
    ]
)
Q = np.matrix(
    [
        [0, 0, 0],
        [0, 0.001, 0],
        [0, 0, 0.001],
    ]
)
R = 10


def plot_results(
    saved_timestamps,
    saved_estimated_positions,
    saved_estimated_velocity,
    saved_estimated_altitude,
):
    fig = make_subplots(
        rows=3,
        cols=1,
        x_title="Time (s)",
        subplot_titles=("x-position", "Velocity", "Altitude"),
        shared_yaxes=True,
    )
    fig.update_layout(title="Radar EKF")

    fig.append_trace(
        plotly_go.Scatter(
            x=saved_timestamps,
            y=saved_estimated_positions,
            name="Estimated x-position",
            mode="markers",
        ),
        row=1,
        col=1,
    )
    fig.update_yaxes(title_text="Estimated x-position", row=1, col=1)

    fig.write_html("chapter14.4_radar_tracking.html")


def run_ekf():
    saved_timestamps = np.zeros(NUM_SAMPLES)
    saved_estimated_positions = np.zeros(NUM_SAMPLES)
    saved_estimated_velocities = np.zeros(NUM_SAMPLES)
    saved_estimated_altitudes = np.zeros(NUM_SAMPLES)

    radar_ekf = RadarEKF(A, Q, R)
    previous_x_estimate = np.matrix(
        [
            [0],
            [90],
            [1100],
        ]
    )
    previous_P_estimate = 10 * np.identity(3)

    for i in range(NUM_SAMPLES):
        z = generate_radar_measurement(DELTA_TIME, previous_x_estimate)

        [x_estimate, P_estimate] = radar_ekf.run_kalman_filter(
            z, previous_x_estimate, previous_P_estimate
        )

        # update for next iteration
        previous_x_estimate = x_estimate
        previous_P_estimate = P_estimate

        # Save results for plotting
        saved_timestamps[i] = i * DELTA_TIME
        saved_estimated_positions[i] = x_estimate[0]
        saved_estimated_velocities[i] = x_estimate[1]
        saved_estimated_altitudes[i] = x_estimate[2]

    return [
        saved_timestamps,
        saved_estimated_positions,
        saved_estimated_velocities,
        saved_estimated_altitudes,
    ]


if __name__ == "__main__":
    [
        saved_timestamps,
        saved_estimated_positions,
        saved_estimated_velocity,
        saved_estimated_altitude,
    ] = run_ekf()

    plot_results(
        saved_timestamps,
        saved_estimated_positions,
        saved_estimated_velocity,
        saved_estimated_altitude,
    )
