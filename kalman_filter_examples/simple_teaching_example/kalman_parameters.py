#!/usr/bin/env python3

"""
Chapter 10.4: Error covariance & Kalman Gain
"""
import numpy as np
import plotly.express as px
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go

# from numpy.linalg import inv
import random

NUM_SAMPLES = 500
DELTA_TIME = 0.2
BATTERY_VOLTAGE = 14

# System model parameters for Kalman Filter
A = np.array([1])  # state transition
H = np.array([1])  # state -> measurement
Q = np.array([0])  # Covariance of state transition noise
R = np.array([2 * 2])  # Covariance of measurement noise


def generate_random_voltage_measurement():
    return BATTERY_VOLTAGE + 4 * random.triangular(-1, 1, 0)


def plot_data(timestamps, measurements, estimates, kalman_gains, error_covariances):
    fig = make_subplots(
        rows=3,
        cols=1,
        x_title="Time (s)",
        subplot_titles=(
            "Measurements & Filtered data",
            "Kalman gain",
            "Error covariance of estimate",
        ),
    )

    fig.append_trace(
        plotly_go.Scatter(x=timestamps, y=measurements, name="Voltage measurement"),
        row=1,
        col=1,
    )
    fig.append_trace(
        plotly_go.Scatter(
            x=timestamps, y=estimates, name="Voltage estimate after noise removal"
        ),
        row=1,
        col=1,
    )
    fig.update_yaxes(title_text="Voltage [V]", row=1, col=1)

    fig.append_trace(
        plotly_go.Scatter(x=timestamps, y=kalman_gains, name="Kalman gain"),
        row=2,
        col=1,
    )
    fig.update_yaxes(title_text="Kalman gain", row=2, col=1)

    fig.append_trace(
        plotly_go.Scatter(
            x=timestamps, y=error_covariances, name="Error covariance of estimate"
        ),
        row=3,
        col=1,
    )
    fig.update_yaxes(title_text="Voltage [V]", row=3, col=1)

    fig.write_html("chapter10.4_error_cov_and_kalman_gain.html")


def inverse_of_1D_matrix(mat):
    # np.linalg.inv does not handle 1D matrix inverse
    return np.array([1 / mat[0]])


def simple_kalman_filter(prev_x_estimate, prev_P_estimate):
    # Step I: Predict state
    x_predict = A * prev_x_estimate
    # Step I: Predict error covariance
    P_predict = A * prev_P_estimate * A.transpose() + Q

    # Step II: Compute Kalman Gain
    K = (
        P_predict
        * H.transpose()
        * inverse_of_1D_matrix(H * P_predict * H.transpose() + R)
    )

    # Step III: Compute estimate
    x_estimate = x_predict + K * (z - H * x_predict)

    # Step IV: Compute error covariance
    P_estimate = P_predict - K * H * P_predict

    return [x_estimate, K, P_estimate]


if __name__ == "__main__":
    saved_timestamps = np.zeros(NUM_SAMPLES)
    saved_measurements = np.zeros(NUM_SAMPLES)
    saved_estimates = np.zeros(NUM_SAMPLES)
    saved_kalman_gain = np.zeros(NUM_SAMPLES)
    saved_error_covariances = np.zeros(NUM_SAMPLES)

    x_estimate = np.array([BATTERY_VOLTAGE])  # Initial state estimate
    P_estimate = np.array(
        [6]
    )  # Make estimated error covariance on the larger size for initial condition

    for i in range(NUM_SAMPLES):
        z = generate_random_voltage_measurement()

        [x_estimate, K, P_estimate] = simple_kalman_filter(x_estimate, P_estimate)

        # Save results for plotting
        saved_timestamps[i] = DELTA_TIME * i
        saved_measurements[i] = z
        saved_estimates[i] = x_estimate[0]
        saved_kalman_gain[i] = K
        saved_error_covariances[i] = P_estimate

    plot_data(
        saved_timestamps,
        saved_measurements,
        saved_estimates,
        saved_kalman_gain,
        saved_error_covariances,
    )
