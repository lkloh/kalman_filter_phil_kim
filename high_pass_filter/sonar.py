#!/usr/bin/env python3

import numpy as np
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
from scipy.io import loadmat

SONAR_MATLAB_DATA = loadmat("../data/SonarAlt.mat")
SONAR_DATA = SONAR_MATLAB_DATA["sonarAlt"][0]
N_SONAR_SAMPLES = len(SONAR_DATA)
DT = 0.01

ALPHA = 0.7
TAU = 1.0 / ALPHA


def plot_results(saved_timestamps, saved_input_measurements, saved_filter_output):
    fig = make_subplots(
        rows=1,
        cols=1,
        x_title="Hihg-pass filter)",
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
            y=saved_filter_output,
            name="Filtered output",
            mode="lines+markers",
        ),
        row=1,
        col=1,
    )
    # fig.update_yaxes(title_text="", row=1, col=1)

    fig.write_html("chapter16.5_high_pass_filter_sonar.html")


def high_pass_filter(input_measurement, prev_input, prev_output):
    coeff = TAU / (TAU + DT)
    return coeff * prev_output + coeff * (input_measurement - prev_input)


def run_filter():
    saved_timestamps = np.zeros(N_SONAR_SAMPLES)
    saved_input_measurements = np.zeros(N_SONAR_SAMPLES)
    saved_filter_output = np.zeros(N_SONAR_SAMPLES)

    prev_input = 0
    prev_output = 0

    for i in range(N_SONAR_SAMPLES):
        input_measurement = SONAR_DATA[i]
        filtered_output = high_pass_filter(input_measurement, prev_input, prev_output)

        # save results
        saved_timestamps[i] = i * DT
        saved_input_measurements[i] = input_measurement
        saved_filter_output[i] = filtered_output

        # Update state
        prev_input = input_measurement
        prev_output = filtered_output

    return [saved_timestamps, saved_input_measurements, saved_filter_output]


if __name__ == "__main__":
    [saved_timestamps, saved_input_measurements, saved_filter_output] = run_filter()
    plot_results(saved_timestamps, saved_input_measurements, saved_filter_output)
