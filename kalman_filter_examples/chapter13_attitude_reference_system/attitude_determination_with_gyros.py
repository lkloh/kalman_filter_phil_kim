#!/usr/bin/env python3

"""
Chapter 13.2:
-----------------------------------------------------------------


"""
import numpy as np
from numpy.linalg import inv
import plotly.express as px
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
import random

NUM_SAMPLES = 100
DELTA_TIME = 0.1

# System model parameters for Kalman Filter

# state transition
A = np.matrix(
    [
        [1, DELTA_TIME, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, DELTA_TIME],
        [0, 0, 0, 1],
    ]
)

# state transition
H = np.matrix([[1, 0, 0, 0], [0, 0, 1, 0]])

# Covariance of state transition noise
Q = np.matrix(
    [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]
)

# Covariance of measurement noise
R = np.matrix(
    [
        [50, 0],
        [0, 50],
    ]
)

FRAME_HEIGHT = 30
FRAME_WIDTH = 50


def get_ball_position(index):
    measurement_x = FRAME_WIDTH - index * (FRAME_WIDTH / NUM_SAMPLES) + random.triangular(-3, 3, 0)
    measurement_y = (
            FRAME_HEIGHT - index * (FRAME_HEIGHT / NUM_SAMPLES) + random.triangular(-4, 4, 0)
    )
    return [measurement_x, measurement_y]

def plot_positions(x_pos_measurements, x_pos_estimates, y_pos_measurements, y_pos_estimates):
    fig = make_subplots(
        rows=1,
        cols=1,
        x_title="x-position",
        y_title="y-position",
    )
    fig.update_layout(title="Ball Position")

    fig.append_trace(
        plotly_go.Scatter(
            x=x_pos_measurements,
            y=y_pos_measurements,
            name="Position measurement",
            mode="markers",
        ),
        row=1,
        col=1,
    )
    fig.append_trace(
        plotly_go.Scatter(
            x=x_pos_estimates,
            y=y_pos_estimates,
            name="Position estimates",
            #mode="markers",
        ),
        row=1,
        col=1,
    )

    fig.write_html("chapter12_tracking_object_in_image_positions.html")

def plot_data_against_time(
    timestamps, x_pos_measurements, x_pos_estimates, y_pos_measurements, y_pos_estimates
):
    fig = make_subplots(
        rows=2,
        cols=1,
        x_title="Time (s)",
        subplot_titles=(
            "x-position",
            "y-position",
        ),
        shared_yaxes=True,
    )
    fig.update_layout(title="Ball Position against time")

    fig.append_trace(
        plotly_go.Scatter(
            x=timestamps,
            y=x_pos_measurements,
            name="x-position measurement",
            mode="markers",
        ),
        row=1,
        col=1,
    )
    fig.append_trace(
        plotly_go.Scatter(x=timestamps, y=x_pos_estimates, name="x-position estimate"),
        row=1,
        col=1,
    )
    fig.update_yaxes(title_text="x-position", row=1, col=1)

    fig.append_trace(
        plotly_go.Scatter(
            x=timestamps,
            y=y_pos_measurements,
            name="y-position measurement",
            mode="markers",
        ),
        row=2,
        col=1,
    )
    fig.append_trace(
        plotly_go.Scatter(x=timestamps, y=y_pos_estimates, name="y-position estimate"),
        row=2,
        col=1,
    )
    fig.update_yaxes(title_text="y-position", row=2, col=1)

    fig.write_html("chapter12_tracking_object_in_image_position_against_time.html")


def kalman_filter(z, prev_x_estimate, prev_P_estimate):
    # Step I: Predict state
    x_predict = A * prev_x_estimate
    # Step I: Predict error covariance
    P_predict = A * prev_P_estimate * A.transpose() + Q

    # Step II: Compute Kalman Gain
    K = P_predict * H.transpose() * inv(H * P_predict * H.transpose() + R)

    # Step III: Compute estimate
    x_estimate = x_predict + K * (z - H * x_predict)

    # Step IV: Compute error covariance
    P_estimate = P_predict - K * H * P_predict

    return [x_estimate, P_estimate]


if __name__ == "__main__":
    tracking_ball()

    timestamps = np.zeros(NUM_SAMPLES)
    x_pos_measurements = np.zeros(NUM_SAMPLES)
    x_pos_estimates = np.zeros(NUM_SAMPLES)
    y_pos_estimates = np.zeros(NUM_SAMPLES)
    y_pos_measurements = np.zeros(NUM_SAMPLES)

    # Initial position of the ball is at (FRAME_WIDTH, FRAME_HEIGHT)
    x_estimate = np.matrix(
        [
            [FRAME_WIDTH],
            [0],
            [FRAME_HEIGHT],
            [0],
        ]
    )

    # Make the initial covariance estimate very large as we have no information about the system yet
    P_estimate = np.matrix(
        [
            [100, 0, 0, 0],
            [0, 100, 0, 0],
            [0, 0, 100, 0],
            [0, 0, 0, 100],
        ]
    )

    for i in range(NUM_SAMPLES):
        # generate data
        [measurement_x, measurement_y] = get_ball_position(i)

        # Estimate positions
        z = np.matrix(
            [
                [measurement_x],
                [measurement_y],
            ]
        )
        [x_estimate, P_estimate] = kalman_filter(z, x_estimate, P_estimate)

        # Save results for plotting
        timestamps[i] = DELTA_TIME * i
        x_pos_measurements[i] = measurement_x
        x_pos_estimates[i] = x_estimate[0]
        y_pos_measurements[i] = measurement_y
        y_pos_estimates[i] = x_estimate[2]

    plot_data_against_time(
        timestamps,
        x_pos_measurements,
        x_pos_estimates,
        y_pos_measurements,
        y_pos_estimates,
    )
    plot_positions(x_pos_measurements,
                   x_pos_estimates,
                   y_pos_measurements,
                   y_pos_estimates)
