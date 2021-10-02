#!/usr/bin/env python3

import numpy as np
import random
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
import retrieve_data as data
from complementary_filter import (
    estimate_attitude_with_complementary_filter,
)
import utils


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

    prev_attitude_estimate = utils.EulerAngles()

    for i in range(data.NUM_GYRO_MEAS):
        accel_meas = data.get_accelerometer_measurements(i)
        gyro_meas = data.get_gyro_measurements(i)

        [phi, theta, psi] = estimate_attitude_with_complementary_filter(
            accel_meas, gyro_meas, data.DELTA_TIME
        )


if __name__ == "__main__":
    run_filter()
    plot_results()
