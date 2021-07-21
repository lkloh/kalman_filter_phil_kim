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
import random

NUM_SAMPLES = 100
ALPHA = 0.5  # 0 < ALPHA < 1
DELTA_TIME = 1


def low_pass_filter(estimate, data_point):
    return ALPHA * estimate + (1 - ALPHA) * data_point


def generate_random_data_point(approximate):
    temp = approximate + 4 * random.triangular(-1, 1, 0)
    return temp * temp


def plot_data(timestamps, measurements, estimates):
    plt.title("Upwards trend data against time")
    plt.xlabel("Time]")
    plt.ylabel("Upwards trend data")
    plt.plot(
        timestamps, measurements, label="Measurement", marker=".", linestyle="None"
    )
    plt.plot(timestamps, estimates, label="Low-pass estimate")
    plt.legend()
    plt.grid()
    plt.show()


if __name__ == "__main__":
    prev_estimate = None

    timestamps = np.zeros(NUM_SAMPLES)
    measurements = np.zeros(NUM_SAMPLES)
    estimates = np.zeros(NUM_SAMPLES)

    for sample_count in range(NUM_SAMPLES):
        data_point = generate_random_data_point(sample_count)

        if prev_estimate is None:
            prev_estimate = data_point
        curr_estimate = low_pass_filter(prev_estimate, data_point)
        print(
            "Low-pass estimate with alpha=%f at sample count %i for data_point %f is %f"
            % (ALPHA, sample_count, data_point, curr_estimate)
        )

        timestamps[sample_count] = DELTA_TIME * sample_count
        measurements[sample_count] = data_point
        estimates[sample_count] = curr_estimate

        prev_estimate = curr_estimate

    plot_data(timestamps, measurements, estimates)
