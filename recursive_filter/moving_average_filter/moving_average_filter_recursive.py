#!/usr/bin/env python3

"""
Chapter 2.3: Recursive moving average filter

There aren't many benefits of doing it this way. as the number of data points
to get the moving average is always WINDOW_DIZE, so the computation
overhead doesn't increase even when a new data point is received. 

We need to store WINDOW_SIZE number of previous data points no matter whether
we use the recursive or naive algorithm.

Recursive implementation is more complicated than the naive implementation,
so unless WINDOW_SIZE is exceptionally large, it is better to use the naive algorithm.
"""
import numpy as np
import random


WINDOW_SIZE = 15
NUM_SAMPLES = 100


def compute_recursive_moving_average_filter(count, buffer, data_point, prev_avg):
    if (prev_avg is None) or (count <= WINDOW_SIZE):
        return data_point

    return prev_avg + (data_point - buffer[0]) / WINDOW_SIZE


if __name__ == "__main__":
    buffer = np.zeros(WINDOW_SIZE + 1)
    prev_avg = None
    for count in range(1, NUM_SAMPLES + 1):
        data_point = random.triangular(-1, 1, 0)

        for i in range(0, WINDOW_SIZE):
            buffer[i] = buffer[i + 1]
        buffer[WINDOW_SIZE] = data_point
        print("\n\n")
        print(buffer)
        curr_avg = compute_recursive_moving_average_filter(
            count, buffer, data_point, prev_avg
        )
        print(
            "Count: %i, Moving average: %f, current data point: %f"
            % (count, curr_avg, data_point)
        )
        prev_avg = curr_avg
