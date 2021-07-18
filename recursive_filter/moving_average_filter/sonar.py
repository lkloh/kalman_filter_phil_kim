#!/usr/bin/env python3

"""
Chapter 2.4: Sonar
==================
Helicopter altitude against time, measured with sonar.
Helicopter vibrations, condition of ground surface and other environment variables
causes noise on the measured data which we remove using moving average filter.

Moving average follows altitude variation well with noise removed.

However there are slight delays from moving average filter,
meaining the actual variation in measured altitude is show with some delays.
* If the delay is too long (e.g. when the quantity to be measured is changing quickly), 
  we should make the moving average window size smaller.
* If the quatity to be measured is changing slowly, we should make the moving
  average window size larger as large window sizes are better at reduces the noise.
"""

from matplotlib import pyplot as plt
import numpy as np
import random
from scipy.io import loadmat

SONAR_MATLAB_DATA = loadmat("SonarAlt.mat")
SONAR_DATA = SONAR_MATLAB_DATA["sonarAlt"][0]
NUM_SONAR_DATA_POINTS = len(SONAR_DATA)
DELTA_TIME = 0.02
WINDOW_SIZE = 10


def plot_data(timestamps, moving_averages):
    plt.title("Altitude [m] against time [s]")
    plt.xlabel("Time [s]")
    plt.ylabel("Altitude [m]")
    plt.plot(timestamps, SONAR_DATA, label="Measurement", marker=".", linestyle="None")
    plt.plot(timestamps, moving_averages, label="Moving average filter")
    plt.legend()
    plt.grid()
    plt.show()


if __name__ == "__main__":
    timestamps = np.zeros(NUM_SONAR_DATA_POINTS)
    moving_averages = np.zeros(NUM_SONAR_DATA_POINTS)

    buffer = np.zeros(WINDOW_SIZE)
    for idx in range(NUM_SONAR_DATA_POINTS):
        data_point = SONAR_DATA[idx]
        count = idx + 1

        for i in range(WINDOW_SIZE - 1):
            buffer[i] = buffer[i + 1]
        buffer[WINDOW_SIZE - 1] = data_point

        moving_avg = sum(buffer) / WINDOW_SIZE

        # save for plotting
        timestamps[idx] = idx * DELTA_TIME
        moving_averages[idx] = moving_avg

    plot_data(timestamps, moving_averages)
