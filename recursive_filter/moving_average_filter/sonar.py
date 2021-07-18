#!/usr/bin/env python3

"""
Chapter 2.4: Sonar
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
