#!/usr/bin/env python3

import numpy as np
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
from scipy.io import loadmat

SONAR_MATLAB_DATA = loadmat("../data/SonarAlt.mat")
SONAR_DATA = SONAR_MATLAB_DATA["sonarAlt"][0]
N_SONAR_SAMPLES = len(SONAR_DATA)
DT = 0.01
TAU = 0.0233


def plot_results(
    saved_timestamps,
    saved_input_measurements,
    saved_filtered_noise,
    saved_measurements_minus_noise,
):
    fig = make_subplots(
        rows=2,
        cols=1,
        x_title="High-pass filter)",
    )
    fig.update_layout(title="")

    fig.append_trace(
        plotly_go.Scatter(
            x=saved_timestamps,
            y=saved_input_measurements,
            name="Input measurements",
            mode="lines+markers",
        ),
        row=1,
        col=1,
    )
    fig.append_trace(
        plotly_go.Scatter(
            x=saved_timestamps,
            y=saved_filtered_noise,
            name="Noise of sonar measurements",
            mode="lines+markers",
        ),
        row=1,
        col=1,
    )
    fig.update_yaxes(title_text="Altitude [m]", row=1, col=1)

    fig.append_trace(
        plotly_go.Scatter(
            x=saved_timestamps,
            y=saved_input_measurements,
            name="Input measurements",
            mode="lines+markers",
        ),
        row=2,
        col=1,
    )
    fig.append_trace(
        plotly_go.Scatter(
            x=saved_timestamps,
            y=saved_measurements_minus_noise,
            name="Measurement - noise (low-pass filter)",
            mode="lines+markers",
        ),
        row=2,
        col=1,
    )
    fig.update_yaxes(title_text="Altitude [m]", row=2, col=1)

    fig.write_html("chapter16.5_high_pass_filter_sonar.html")


def high_pass_filter(input_measurement, prev_input, prev_output):
    coeff = TAU / (TAU + DT)
    return coeff * prev_output + coeff * (input_measurement - prev_input)


def run_filter():
    saved_timestamps = np.zeros(N_SONAR_SAMPLES)
    saved_measurements = np.zeros(N_SONAR_SAMPLES)
    saved_noise = np.zeros(N_SONAR_SAMPLES)
    saved_measurements_minus_noise = np.zeros(N_SONAR_SAMPLES)

    prev_input = 0
    prev_output = 0

    for i in range(N_SONAR_SAMPLES):
        input_measurement = SONAR_DATA[i]
        filtered_noise = high_pass_filter(input_measurement, prev_input, prev_output)

        # save results
        saved_timestamps[i] = i * DT
        saved_measurements[i] = input_measurement
        saved_noise[i] = filtered_noise
        saved_measurements_minus_noise[i] = (input_measurement - filtered_noise)

        # Update state
        prev_input = input_measurement
        prev_output = filtered_noise

    return [
        saved_timestamps,
        saved_measurements,
        saved_noise,
        saved_measurements_minus_noise,
    ]


if __name__ == "__main__":
    [
        saved_timestamps,
        saved_input_measurements,
        saved_filtered_noise,
        saved_measurements_minus_noise,
    ] = run_filter()
    plot_results(
        saved_timestamps,
        saved_input_measurements,
        saved_filtered_noise,
        saved_measurements_minus_noise,
    )
