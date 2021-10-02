#!/usr/bin/env python3

import numpy as np
import random
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
import retrieve_data as data


def plot_results():
    pass


def run_filter():
    if data.NUM_GYRO_MEAS != data.NUM_ACCEL_MEAS:
        print(
            "Number of gyroscope measurements ({}) is not equal to number of accelerometer measurements ({})".format(
                data.NUM_GYRO_MEAS, data.NUM_ACCEL_MEAS
            )
        )
        exit(1)


if __name__ == "__main__":
    run_filter()
    plot_results()
