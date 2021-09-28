#!/usr/bin/env python3

import numpy as np
import random
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go

N_SAMPLES = 1000
DT = 0.01
ALPHA = 0.7
TAU = 1.0 / ALPHA


def generate_meas():
    return 15 + 2 * random.triangular(-1, 1, 0)


def high_pass_filter(input_measurement, prev_input, prev_output):
    coeff = TAU / (TAU + DT)
    return coeff * prev_output + coeff * (input_measurement - prev_input)


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

    fig.write_html("chapter16.4_high_pass_filter_function.html")


def run_filter():
    saved_timestamps = np.zeros(N_SAMPLES)
    saved_input_measurements = np.zeros(N_SAMPLES)
    saved_filter_output = np.zeros(N_SAMPLES)

    prev_output = 0
    prev_input = 0
    for i in range(N_SAMPLES):
        input_measurement = generate_meas()
        filtered_output = high_pass_filter(input_measurement, prev_input, prev_output)

        # save results
        saved_timestamps[i] = i * DT
        saved_input_measurements[i] = input_measurement
        saved_filter_output[i] = filtered_output

        # Update
        prev_input = input_measurement
        prev_output = filtered_output

    return [saved_timestamps, saved_input_measurements, saved_filter_output]


if __name__ == "__main__":
    [saved_timestamps, saved_input_measurements, saved_filter_output] = run_filter()
    plot_results(saved_timestamps, saved_input_measurements, saved_filter_output)
