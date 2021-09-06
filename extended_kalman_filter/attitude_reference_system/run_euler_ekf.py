#!/usr/bin/env python3

import numpy as np
import math
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
import retrieve_data as meas


def run_ekf():
    saved_phi = np.zeros(meas.NUM_SAMPLES)
    saved_theta = np.zeros(meas.NUM_SAMPLES)
    saved_psi = np.zeros(meas.NUM_SAMPLES)

    for idx in range(meas.NUM_SAMPLES):
        angular_vel = meas.get_gyro_measurements(idx)


if __name__ == "__main__":
    run_ekf()
