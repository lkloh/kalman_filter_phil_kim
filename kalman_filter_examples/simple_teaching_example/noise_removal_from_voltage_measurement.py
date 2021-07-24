#!/usr/bin/env python3

"""
Chapter 10: Extremely Simple Example
"""
from matplotlib import pyplot as plt
import numpy as np
import random

NUM_SAMPLES = 100
DELTA_TIME = 0.2
BATTERY_VOLTAGE = 14

# System model parameters for Kalman Filter
A = 1  # state transition
H = 1  # state -> measurement
Q = 0  # Covariance of state transition noise
R = 2 * 2  # Covariance of measurement noise


def generate_random_voltage_measurement():
    return BATTERY_VOLTAGE + 4 * random.triangular(-1, 1, 0)


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


def simple_kalman_filter(prev_x_estimate, prev_P_estimate):
    A_T = A
    A_inv = A
    H_T = H

    # Step I: Predict state
    x_predict = A * prev_x_estimate
    # Step I: Predict error covariance
    P_predict = A * prev_P_estimate * A_T + Q

    # Step II: Compute Kalman Gain
    K = P_predict * H_T * (1 / (H * P_predict * H_T + R))

    # Step III: Compute estimate
    x_estimate = x_predict + K * (z - H * x_predict)

    # Step IV: Compute error covariance
    P_estimate = P_predict - K * H * P_predict

    return [x_estimate, P_estimate]


if __name__ == "__main__":
    saved_timestamps = np.zeros(NUM_SAMPLES)
    saved_measurements = np.zeros(NUM_SAMPLES)
    saved_estimates = np.zeros(NUM_SAMPLES)

    x_estimate = BATTERY_VOLTAGE  # Initial state estimate
    P_estimate = (
        6  # Make estimated error covariance on the large size for initial condition
    )

    for i in range(NUM_SAMPLES):
        z = generate_random_voltage_measurement()

        [x_estimate, P_estimate] = simple_kalman_filter(x_estimate, P_estimate)

        # Save results for plotting
        saved_timestamps[i] = DELTA_TIME * i
        saved_measurements[i] = z
        saved_estimates[i] = x_estimate

    plot_data(saved_timestamps, saved_measurements, saved_estimates)
