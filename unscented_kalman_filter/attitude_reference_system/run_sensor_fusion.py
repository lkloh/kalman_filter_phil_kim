#!/usr/bin/env python3

import numpy as np
import math
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
import retrieve_data as ats


if __name__ == "__main__":
    if ats.NUM_GYRO_MEAS != ats.NUM_ACCEL_MEAS:
        print(
            "Number of gyroscope measurements not equal to number of accelerometer measurements, exiting."
        )
        exit(1)

    for i in range(ats.NUM_GYRO_MEAS):
        gyro_meas = ats.get_gyro_measurements(i)
        print(gyro_meas.p)
