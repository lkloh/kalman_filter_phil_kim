#!/usr/bin/env python3

"""
Chapter 11: Estimating velocity from position with the Kalman Filter
--------------------------------------------------------------------

We again have a train that is supposed to maintain a velocity of 80m/s
on a straight track. The position is measured and stored very 0.1 seconds.
We want to obtain the velocity as well.

The low-pass filter and Kalman filter can both be used for noise removal
from the position measurements, but only the Kalman filter can be used to
estimate velocity from the measured position data.

Challenges with estimating velocity from measured position data:
* estimated_velocity = measured_displacement / time_of_travel
* Using the ussual formular results in large errors in estimated velocity.
  Noise from the position data and small elapsed time results in a large
  spike when performing differentiation.
* Using the moving average of position data to approximate the position
  measurement and then differentiating would help, but it's complicated
  and does not fit data well.

Designing the Kalman Filter to estimate velocity from position:
* Physical quantities we are interested in: position & velocity.
  Set them as state variables.
  x = [position]
      [velocity]
* System model - derived from physics equations / refer to similar examples
  x_{k+1} = A * x_k + w_k
  z_k = H * x_k + v_k
  A = [1 \delta(t)]
      [0    1    ]
  H = [0 1]
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
from matplotlib import pyplot as plt
import numpy as np

# from numpy.linalg import inv
import random

NUM_SAMPLES = 100
DELTA_TIME = 0.2
BATTERY_VOLTAGE = 14

# System model parameters for Kalman Filter
A = np.array([1])  # state transition
H = np.array([1])  # state -> measurement
Q = np.array([0])  # Covariance of state transition noise
R = np.array([2 * 2])  # Covariance of measurement noise


def generate_random_position_measurement():
    return 1


def plot_data(timestamps, measurements, estimates):
    plt.title("Voltage [V] against time [s]")
    plt.xlabel("Time [s]")
    plt.ylabel("Voltage [V]")
    plt.plot(
        timestamps, measurements, label="Measurement", marker=".", linestyle="dotted"
    )
    plt.plot(timestamps, estimates, label="Kalman filter estimate")
    plt.legend()
    plt.grid()
    plt.show()


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

    return [x_estimate, P_estimate]


if __name__ == "__main__":
    saved_timestamps = np.zeros(NUM_SAMPLES)
    saved_pos_measurements = np.zeros(NUM_SAMPLES)
    saved_vel_estimates = np.zeros(NUM_SAMPLES)

    x_estimate = np.array([BATTERY_VOLTAGE])  # Initial state estimate
    P_estimate = np.array(
        [6]
    )  # Make estimated error covariance on the larger size for initial condition

    for i in range(NUM_SAMPLES):
        z = generate_random_position_measurement()

        [x_estimate, P_estimate] = simple_kalman_filter(x_estimate, P_estimate)

        # Save results for plotting
        saved_timestamps[i] = DELTA_TIME * i
        saved_pos_measurements[i] = z
        saved_vel_estimates[i] = x_estimate[0]

    plot_data(saved_timestamps, saved_pos_measurements, saved_vel_estimates)
    