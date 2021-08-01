#!/usr/bin/env python3

"""
Chapter 11.4: Estimating position with velocity with the Kalman Filter
----------------------------------------------------------------------

We have a train that is supposed to maintain a velocity of 80m/s
on a straight track. The velocity is measured and stored every 0.1 seconds.
We want to derive the position from the velocity measurements.

Designing the Kalman Filter to estimate velocity from position:
* Physical quantities we are interested in: position & velocity.
  Set them as state variables.
  x = [position]
      [velocity]
* System model - derived from physics equations / refer to similar examples
  x_{k+1} = A * x_k + w_k
  z_k = H * x_k + v_k
  A = [1 \delta(t)]
      [0      1   ]
  H = [0   1]
  Where \delta(t) is the interval of time measuring the velocity.
* Verifying the system model represents the system appropriately:
  [position_{k+1}] = [1 \delta(t)][position_k] + [ 0 ]
  [velocity_{k+1}]   [0      1   ][velocity_k]   [w_k]
                   = [position_k + velocity_k * \delta(t) ]
                     [velocity_k + w_k                    ]
  We see that the position keeps increasing since
     position_{k+1} = position_k + velocity_k * \delta(t)
  by the velocity stays the same:
     velocity_{k+1} = velocity_k + w_k
  as the velocity of the train affected only by system noise
  (e.g. friction, error in the engine controller), but no external forces
  are acting on the train.
* To see the measurement equation of the system model:
  z_k = H * x_k + v_k
      = [0  1][position_k] + v_k
              [velocity_k]
      = velocity_k + v_k
  so the measured position z_k is the sum of the true position (position_k)
  and the measurement noise (v_k).
* The measurement noise covariance matrix R is usually provided by the sensor manufacturer.
  If not available it must by determined experimentally.
* The state transition noise covariance matrix Q is harder to determine and
  requires knowledge and experience about the system. If it cannot be defined
  analytically then it must be defined through trial and error.
"""
import numpy as np
from numpy.linalg import inv
import plotly.express as px
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
import random

NUM_SAMPLES = 50
DELTA_TIME = 0.1
PREPROGRAMMED_VELOCITY_OF_TRAIN = 80

# System model parameters for Kalman Filter

# state transition
A = np.matrix([[1, DELTA_TIME], [0, 1]])

# state transition
H = np.matrix([0, 1])

# Covariance of state transition noise
Q = np.matrix([[1, 0], [0, 3]])

# Covariance of measurement noise
R = np.matrix([10])


def generate_random_velocity_measurement(prev_true_position, prev_true_velocity):
    noise_of_true_position = random.triangular(-3, 3, 0)
    noise_of_true_velocity = random.triangular(-10, 10, 0)
    true_position = prev_true_position + (prev_true_velocity * DELTA_TIME) - noise_of_true_position
    true_velocity = PREPROGRAMMED_VELOCITY_OF_TRAIN + noise_of_true_velocity
    measured_velocity = true_velocity
    return [true_position, true_velocity, measured_velocity]


def plot_data(
    timestamps,
    true_positions,
    position_estimates,
    velocity_measurements,
    velocity_estimates,
):
    fig = make_subplots(
        rows=2,
        cols=1,
        x_title="Time (s)",
        subplot_titles=(
            "Position",
            "Velocity",
        ),
        shared_yaxes=True,
    )
    fig.update_layout(
        title="Position & Velocity estimates of a train programed to move on a straight track at {} m/s".format(
            str(PREPROGRAMMED_VELOCITY_OF_TRAIN)
        )
    )

    fig.append_trace(
        plotly_go.Scatter(
            x=timestamps,
            y=true_positions,
            name="True position",
            mode="markers",
        ),
        row=1,
        col=1,
    )
    fig.append_trace(
        plotly_go.Scatter(x=timestamps, y=position_estimates, name="Position estimate"),
        row=1,
        col=1,
    )
    fig.update_yaxes(title_text="Position", row=1, col=1)

    fig.append_trace(
        plotly_go.Scatter(
            x=timestamps,
            y=velocity_measurements,
            name="Measured velocity",
            mode="markers",
        ),
        row=2,
        col=1,
    )
    fig.append_trace(
        plotly_go.Scatter(x=timestamps, y=velocity_estimates, name="Velocity estimate"),
        row=2,
        col=1,
    )
    fig.update_yaxes(title_text="Velocity", row=2, col=1)

    fig.write_html("chapter11_4_estimating_position_from_velocity.html")


def kalman_filter(prev_x_estimate, prev_P_estimate):
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
    saved_timestamps = np.zeros(NUM_SAMPLES)
    saved_true_position = np.zeros(NUM_SAMPLES)
    saved_position_estimates = np.zeros(NUM_SAMPLES)
    saved_velocity_measurement = np.zeros(NUM_SAMPLES)
    saved_velocity_estimates = np.zeros(NUM_SAMPLES)

    # Initial state estimate - velocity is wildly off!!
    x_estimate = np.matrix([[0], [20]])

    # Make estimated error covariance on the larger size for initial condition
    P_estimate = np.matrix([[5, 0], [0, 5]])

    prev_true_position = 0
    prev_true_velocity = PREPROGRAMMED_VELOCITY_OF_TRAIN

    for i in range(NUM_SAMPLES):
        ## generate data
        [true_position, true_velocity, z] = generate_random_velocity_measurement(
            prev_true_position, prev_true_velocity
        )
        prev_true_position = true_position
        prev_true_velocity = true_velocity

        [x_estimate, P_estimate] = kalman_filter(x_estimate, P_estimate)

        # Save results for plotting
        saved_timestamps[i] = DELTA_TIME * i
        saved_true_position[i] = true_position
        saved_position_estimates[i] = x_estimate[0]
        saved_velocity_measurement[i] = z
        saved_velocity_estimates[i] = x_estimate[1]

    plot_data(
        saved_timestamps,
        saved_true_position,
        saved_position_estimates,
        saved_velocity_measurement,
        saved_velocity_estimates,
    )
