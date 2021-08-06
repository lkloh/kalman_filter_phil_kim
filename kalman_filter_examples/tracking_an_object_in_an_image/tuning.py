#!/usr/bin/env python3

"""
Chapter 12: Tuning the Kalman filter
------------------------------------
We try to improve the performance of the Kalman filter by tuning the matrices
Q (system transition covariance) and R (measurement covariance).

Recall that the formula for Kalman Gain is:
    K_k = P_k^- * H^T * (H * P_k^- * H^T + R)^{-1}

and that the prediction of the error covariance P_{k+1}^- is:
    P_{k+1}^- = A * P_k * A^T + Q

Matrices are hard to understand, so re-write the formula for Kalman Gain as:
               P_k^- * H^T
    K_k = ---------------------
          (H * P_k^- * H^T) + R

As R (measurement covariance) increases, then the Kalman Gain decreases.

Since the formula for the Kalman estimate is
    \hat{x}_k = (1 - K_k) * \hat{x}_k^- + K_k * z_k,
when the Kalman Gain decreases then the previous estimate \hat{x}_k^-
has greater importance in the new estimate and the measurement x_k has
less importance. So the noise in the measurement is also less influential
and the estimate gets smoother.

The system transition noise Q is used to get the prediction of the
error covariance P_{k+1}^-:
    P_{k+1}^- = A * P_k * A^T + Q
As Q increases, so does the prediction of the error covariance P_{k+1}^-.
By the formula for the Kalman Gain
               P_k^- * H^T
    K_k = ---------------------
          (H * P_k^- * H^T) + R
then P_{k+1}^- appears in both the numerator and denominator, but the
numerator increases slightly faster as R is added to the denominator.
Thus when Q increases, Kalman gain K_k increases, meaning that the
measurement is weighted more strongly in the new estimate.
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
large_Q = np.matrix(
    [
        [50, 0, 0, 0],
        [0, 50, 0, 0],
        [0, 0, 50, 0],
        [0, 0, 0, 50],
    ]
)
small_Q = np.matrix(
    [
        [0.1, 0, 0, 0],
        [0, 0.1, 0, 0],
        [0, 0, 0.1, 0],
        [0, 0, 0, 0.1],
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
    measurement_x = (
        FRAME_WIDTH - index * (FRAME_WIDTH / NUM_SAMPLES) + random.triangular(-3, 3, 0)
    )
    measurement_y = (
        FRAME_HEIGHT
        - index * (FRAME_HEIGHT / NUM_SAMPLES)
        + random.triangular(-4, 4, 0)
    )
    return [measurement_x, measurement_y]


def plot_positions(
    x_pos_measurements,
    y_pos_measurements,
    x_pos_estimates_large_Q,
    y_pos_estimates_large_Q,
    x_pos_estimates_small_Q,
    y_pos_estimates_small_Q,
):
    fig = make_subplots(
        rows=2,
        cols=1,
        x_title="x-position",
        y_title="y-position",
        subplot_titles=(
            "Large Q (measurements are highly weighted), jagged estimate",
            "Small Q (previous estimate is highly weighted), smooth estimate",
        ),
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
            x=x_pos_estimates_large_Q,
            y=y_pos_estimates_large_Q,
            name="Position estimates, large Q",
            # mode="markers",
        ),
        row=1,
        col=1,
    )

    fig.append_trace(
        plotly_go.Scatter(
            x=x_pos_measurements,
            y=y_pos_measurements,
            name="Position measurement",
            mode="markers",
        ),
        row=2,
        col=1,
    )
    fig.append_trace(
        plotly_go.Scatter(
            x=x_pos_estimates_small_Q,
            y=y_pos_estimates_small_Q,
            name="Position estimates, small Q",
            # mode="markers",
        ),
        row=2,
        col=1,
    )

    fig.write_html("chapter12_tuning.html")


def kalman_filter(z, prev_x_estimate, prev_P_estimate, Q):
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
    x_pos_measurements = np.zeros(NUM_SAMPLES)
    y_pos_measurements = np.zeros(NUM_SAMPLES)

    x_pos_estimates_large_Q = np.zeros(NUM_SAMPLES)
    y_pos_estimates_large_Q = np.zeros(NUM_SAMPLES)

    x_pos_estimates_small_Q = np.zeros(NUM_SAMPLES)
    y_pos_estimates_small_Q = np.zeros(NUM_SAMPLES)

    # Initial position of the ball is at (FRAME_WIDTH, FRAME_HEIGHT)
    x_estimate_large_Q = np.matrix(
        [
            [FRAME_WIDTH],
            [0],
            [FRAME_HEIGHT],
            [0],
        ]
    )
    # Make the initial covariance estimate very large as we have no information about the system yet
    P_estimate_large_Q = 100 * large_Q

    # Initial position of the ball is at (FRAME_WIDTH, FRAME_HEIGHT)
    x_estimate_small_Q = np.matrix(
        [
            [FRAME_WIDTH],
            [0],
            [FRAME_HEIGHT],
            [0],
        ]
    )
    # Make the initial covariance estimate very large as we have no information about the system yet
    P_estimate_small_Q = 100 * small_Q

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
        [x_estimate_large_Q, P_estimate_large_Q] = kalman_filter(
            z, x_estimate_large_Q, P_estimate_large_Q, large_Q
        )
        [x_estimate_small_Q, P_estimate_small_Q] = kalman_filter(
            z, x_estimate_small_Q, P_estimate_small_Q, small_Q
        )

        # Save results for plotting
        x_pos_measurements[i] = measurement_x
        y_pos_measurements[i] = measurement_y
        x_pos_estimates_large_Q[i] = x_estimate_large_Q[0]
        y_pos_estimates_large_Q[i] = x_estimate_large_Q[2]
        x_pos_estimates_small_Q[i] = x_estimate_small_Q[0]
        y_pos_estimates_small_Q[i] = x_estimate_small_Q[2]

    plot_positions(
        x_pos_measurements,
        y_pos_measurements,
        x_pos_estimates_large_Q,
        y_pos_estimates_large_Q,
        x_pos_estimates_small_Q,
        y_pos_estimates_small_Q,
    )
