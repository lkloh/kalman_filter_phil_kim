#!/usr/bin/env python3

"""
Chapter 1.3: Voltage measurement example


Notice that the average value is much more stable than the randomly generated
measurement values. As data accumulates, the output value of `compute_recursive_average`
approaches the actual average value of the voltage.

When the physical quantity measured is average, its noise gets filtered out.
This is how the equation

\bar{x}_k = \alpha\bar{x}_{k-1} + (1-\alpha)x_k
where \alpha = \frac{k-1}{k}

came to be known as the "average filter"
"""

from matplotlib import pyplot as plt
import numpy as np
import random

NUM_SAMPLES = 200
DELTA_TIME = 0.2


def compute_recursive_average(count, data_point, prev_avg):
    if prev_avg is None:
        count = 1
        prev_avg = 0

    alpha = (count - 1) / count
    return alpha * prev_avg + (1 - alpha) * data_point


def get_random_voltage_measurement():
    AVG = 14.4
    STD = 4
    return AVG + STD * random.triangular(-4, 4, 0)


if __name__ == "__main__":
    prev_avg = None

    timestamps = np.zeros(NUM_SAMPLES)
    saved_averages = np.zeros(NUM_SAMPLES)
    saved_measurements = np.zeros(NUM_SAMPLES)

    for idx in range(0, NUM_SAMPLES):
        measurement = get_random_voltage_measurement()
        curr_avg = compute_recursive_average(idx + 1, measurement, prev_avg)

        saved_measurements[idx] = measurement
        timestamps[idx] = idx * DELTA_TIME
        saved_averages[idx] = curr_avg
        prev_avg = curr_avg

    plt.title("Voltage [V] against time [s]")
    plt.xlabel("Time [s]")
    plt.ylabel("Voltage [V]")
    plt.plot(timestamps, saved_measurements, label="Measurement", marker=".")
    plt.plot(timestamps, saved_averages, label="Average", marker=".")
    plt.legend()
    plt.grid()
    plt.show()
