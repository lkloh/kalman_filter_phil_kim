#!/usr/bin/env python3

'''
Chapter 1.2: Average filter function
'''


def average_recursive_filter(count, data_point, prev_avg):
    if prev_avg is None:
        count = 1
        prev_avg = 1

    alpha = (count - 1) / count
    return alpha * prev_avg + (1 - alpha) * data_point


if __name__ == "__main__":
    prev_avg = None
    for count in range(1, 11):
        curr_avg = average_recursive_filter(count, count, prev_avg)
        print(
            "\nWhen count is %i and data point is %i, average is %f"
            % (count, count, curr_avg)
        )

        prev_avg = curr_avg
