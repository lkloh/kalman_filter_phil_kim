#!/usr/bin/env python3

"""
Chapter 2.3: Moving average filter - naive algorithm

"""
import numpy as np
import random


WINDOW_SIZE = 15
NUM_SAMPLES = 100


if __name__ == "__main__":
    buffer = np.zeros(WINDOW_SIZE)

    for count in range(1, NUM_SAMPLES + 1):
        data_point = random.triangular(-1, 1, 0)

        for i in range(0, WINDOW_SIZE - 1):
            buffer[i] = buffer[i + 1]
        buffer[WINDOW_SIZE - 1] = data_point

        moving_avg = data_point if count < WINDOW_SIZE else (sum(buffer) / WINDOW_SIZE)

        print(
            "Count: %i, Moving average: %f, current data point: %f"
            % (count, moving_avg, data_point)
        )
