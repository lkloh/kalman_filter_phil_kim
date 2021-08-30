#!/usr/bin/env python3

import numpy as np
import math
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
from scipy.io import loadmat
from get_radar import generate_radar_measurement
from radar_ekf import run_radar_ekf

NUM_SAMPLES = 100
DELTA_TIME = 0.5


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

    predicted_x_position = 0
    for i in range(NUM_SAMPLES):
        [radar_measurement, estimated_x_position] = generate_radar_measurement(
            DELTA_TIME, predicted_x_position
        )
        predicted_x_position = estimated_x_position

        [estimated_pos, estimated_vel, estimated_alt] = run_radar_ekf(
            radar_measurement, NUM_SAMPLES, DELTA_TIME
        )

        saved_timestamps[i] = i * DELTA_TIME
        saved_estimated_positions[i] = estimated_pos
        saved_estimated_velocities[i] = estimated_vel
        saved_estimated_altitudes[i] = estimated_alt

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
