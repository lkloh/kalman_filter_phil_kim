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


def plot_radar(saved_timestamps, saved_measured_radar, saved_estimated_radar):
    fig = make_subplots(
        rows=1,
        cols=1,
        x_title="Time (s)",
    )
    fig.update_layout(title="Radar EKF")

    fig.append_trace(
        plotly_go.Scatter(
            x=saved_timestamps,
            y=saved_estimated_radar,
            name="Estimated radar",
            mode="lines+markers",
        ),
        row=1,
        col=1,
    )
    fig.append_trace(
        plotly_go.Scatter(
            x=saved_timestamps,
            y=saved_measured_radar,
            name="Measured radar",
            mode="lines+markers",
        ),
        row=1,
        col=1,
    )
    fig.update_yaxes(title_text="Estimated x-position", row=1, col=1)

    fig.write_html("chapter14.4_measured_vs_estimated_radar_tracking.html")


def plot_state_variables(
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
    fig.update_layout(title="Radar EKF - state variables")

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

    fig.append_trace(
        plotly_go.Scatter(
            x=saved_timestamps,
            y=saved_estimated_velocity,
            name="Estimated velocity",
            mode="markers",
        ),
        row=2,
        col=1,
    )
    fig.update_yaxes(title_text="Estimated velocity", row=2, col=1)

    fig.append_trace(
        plotly_go.Scatter(
            x=saved_timestamps,
            y=saved_estimated_altitude,
            name="Estimated altitude",
            mode="markers",
        ),
        row=3,
        col=1,
    )
    fig.update_yaxes(title_text="Estimated altitude", row=3, col=1)

    fig.write_html("chapter14.4_radar_tracking_state_vars.html")


def run_ekf():
    saved_timestamps = np.zeros(NUM_SAMPLES)
    saved_measured_radar = np.zeros(NUM_SAMPLES)
    saved_estimated_radar = np.zeros(NUM_SAMPLES)
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

        # Save results for plotting
        estimated_xpos = x_estimate[0, 0]
        estimate_vel = x_estimate[1, 0]
        estimated_alt = x_estimate[2, 0]
        saved_timestamps[i] = i * DELTA_TIME
        saved_measured_radar[i] = z
        saved_estimated_radar[i] = math.sqrt(
            math.pow(estimated_xpos, 2) + math.pow(estimated_alt, 2)
        )
        saved_estimated_positions[i] = estimated_xpos
        saved_estimated_velocities[i] = estimate_vel
        saved_estimated_altitudes[i] = estimated_alt

        # update for next iteration
        previous_x_estimate = x_estimate
        previous_P_estimate = P_estimate

    return [
        saved_timestamps,
        saved_measured_radar,
        saved_estimated_radar,
        saved_estimated_positions,
        saved_estimated_velocities,
        saved_estimated_altitudes,
    ]


if __name__ == "__main__":
    [
        saved_timestamps,
        saved_measured_radar,
        saved_estimated_radar,
        saved_estimated_positions,
        saved_estimated_velocity,
        saved_estimated_altitude,
    ] = run_ekf()

    plot_radar(saved_timestamps, saved_measured_radar, saved_estimated_radar)
    plot_state_variables(
        saved_timestamps,
        saved_estimated_positions,
        saved_estimated_velocity,
        saved_estimated_altitude,
    )
