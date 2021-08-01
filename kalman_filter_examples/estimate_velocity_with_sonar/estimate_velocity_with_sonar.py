#!/usr/bin/env python3

"""
Chapter 11.5: Estimate velocity with sonar altitude measurements
----------------------------------------------------------------

Suppose we are developing a sonar. We install a prototype to a
helicopter. The helicoptor makes a test flight and measures vertical
distance to the ground surface, storing measurements at an interval
of 0.02 s. Helicopter vibrations, condition of the ground surface,
and other environmental factors contribute to measurement noise.

The helicopter moved up an down with varying velocity. We want to
verify that the Kalman filter could estimate velocity from the
altitude data even when velocity is not constant.

Designing the Kalman Filter to estimate velocity from position -
we use the same principles as before as physical laws describing
relation between distance and velocity does not change.

* Physical quantities we are interested in: position & velocity.
  Set them as state variables.
  x = [position]
      [velocity]
* System model - derived from physics equations / refer to similar examples
  x_{k+1} = A * x_k + w_k
  z_k = H * x_k + v_k
  A = [1 \delta(t)]
      [0    1    ]
  H = [1 0]
  Where \delta(t) is the interval of time measuring the position.
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
      = [1 0][position_k] + v_k
             [velocity_k]
      = position_k + v_k
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
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
from scipy.io import loadmat

SONAR_MATLAB_DATA = loadmat("../../data/SonarAlt.mat")
SONAR_DATA = SONAR_MATLAB_DATA["sonarAlt"][0]
NUM_SONAR_DATA_POINTS = len(SONAR_DATA)
DELTA_TIME = 0.02

# System model parameters for Kalman Filter

# state transition
A = np.matrix([[1, DELTA_TIME], [0, 1]])

# state transition
H = np.matrix([1, 0])

# Covariance of state transition noise
Q = np.matrix([[1, 0], [0, 3]])

# Covariance of measurement noise
R = np.matrix([10])


def plot_data(
    timestamps,
    position_measurements,
    position_estimates,
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
        title="Position & Velocity estimates of a helicopter moving at varying speeds"
    )

    fig.append_trace(
        plotly_go.Scatter(
            x=timestamps,
            y=position_measurements,
            name="Position measurement",
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
            y=position_measurements,
            name="Position measurement",
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

    fig.write_html("chapter11_5_estimating_velocity_from_sonar.html")


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

    saved_timestamps = np.zeros(NUM_SONAR_DATA_POINTS)
    saved_position_measurements = np.zeros(NUM_SONAR_DATA_POINTS)
    saved_position_estimates = np.zeros(NUM_SONAR_DATA_POINTS)
    saved_velocity_estimates = np.zeros(NUM_SONAR_DATA_POINTS)

    # Initial state estimate - velocity is wildly off!!
    x_estimate = np.matrix([[0], [20]])

    # Make estimated error covariance on the larger size for initial condition
    P_estimate = np.matrix([[5, 0], [0, 5]])

    prev_true_position = 0
    prev_true_velocity = x_estimate[1]

    for i in range(NUM_SONAR_DATA_POINTS):
        z = SONAR_DATA[i]

        [x_estimate, P_estimate] = kalman_filter(x_estimate, P_estimate)

        # Save results for plotting
        saved_timestamps[i] = DELTA_TIME * i
        saved_position_measurements[i] = z
        saved_position_estimates[i] = x_estimate[0]
        saved_velocity_estimates[i] = x_estimate[1]

    plot_data(
        saved_timestamps,
        saved_position_measurements,
        saved_position_estimates,
        saved_velocity_estimates,
    )
