#!/usr/bin/env python3

'''
Chapter 1.2: Average filter function

Average filter is a recursive expression give by

\bar{x}_k = \alpha\bar{x}_{k-1} + (1-\alpha)x_k
where \alpha=\frac{k-1}{k}

The average \bar{x}_k of a sequence x_1, x_2, ..., x_k is obtained from the previous average
\bar{x}_{k-1} of the sequence x_1, x_2, ..., x_{k-1}, the count of datums k, and the 
new data point x_k. This computation is very efficient when the data points are input sequentially,
and is especially useful when processing real-time data.
'''


def average_filter(count, data_point, prev_avg):
    if prev_avg is None:
        count = 1
        prev_avg = 1

    alpha = (count - 1) / count
    return alpha * prev_avg + (1 - alpha) * data_point


if __name__ == "__main__":
    prev_avg = None
    for count in range(1, 11):
        curr_avg = average_filter(count, count, prev_avg)
        print(
            "\nWhen count is %i and data point is %i, average is %f"
            % (count, count, curr_avg)
        )

        prev_avg = curr_avg
