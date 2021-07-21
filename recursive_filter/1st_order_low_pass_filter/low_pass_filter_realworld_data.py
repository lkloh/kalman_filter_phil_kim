#!/usr/bin/env python3

"""
Chapter 3.2: 1st order low-pass filter

\bar{x}_k = \alpha\bar_{x}_{k-1} + (1-\alpha)x_k 
where 0 < \alpha < 1

More recent measurements are more highly-weighed than less recent ones.
This enables noise-filtering and also tracking variation in the measurements simultaneously
with minimal time-delay.
"""
from matplotlib import pyplot as plt
import numpy as np
from scipy.io import loadmat

ALPHA = 0.1  # 0 < ALPHA < 1

SONAR_MATLAB_DATA = loadmat("../SonarAlt.mat")
SONAR_DATA = SONAR_MATLAB_DATA["sonarAlt"][0]
NUM_SONAR_DATA_POINTS = len(SONAR_DATA)
DELTA_TIME = 0.02
WINDOW_SIZE = 10


def low_pass_filter(estimate, data_point):
    return ALPHA * estimate + (1 - ALPHA) * data_point


def plot_data(timestamps, measurements, estimates):
    plt.title("Altitude [m] against time [s] with alpha=%f" % (ALPHA))
    plt.xlabel("Time [s]")
    plt.ylabel("Altitude [m]")
    plt.plot(
        timestamps, measurements, label="Measurement", marker=".", linestyle="None"
    )
    plt.plot(timestamps, estimates, label="Low-pass estimate")
    plt.legend()
    plt.grid()
    plt.show()


if __name__ == "__main__":
    prev_estimate = None

    timestamps = np.zeros(NUM_SONAR_DATA_POINTS)
    measurements = np.zeros(NUM_SONAR_DATA_POINTS)
    estimates = np.zeros(NUM_SONAR_DATA_POINTS)

    for sample_count in range(NUM_SONAR_DATA_POINTS):
        data_point = SONAR_DATA[sample_count]

        if prev_estimate is None:
            prev_estimate = data_point
        curr_estimate = low_pass_filter(prev_estimate, data_point)

        timestamps[sample_count] = DELTA_TIME * sample_count
        measurements[sample_count] = data_point
        estimates[sample_count] = curr_estimate

        prev_estimate = curr_estimate

    plot_data(timestamps, measurements, estimates)
