#!/usr/bin/env python3

import numpy as np
import math
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
import retrieve_data as meas
import euler_ekf


def run_ekf():
    saved_timestamps = np.zeros(meas.NUM_SAMPLES)
    saved_phi = np.zeros(meas.NUM_SAMPLES)
    saved_theta = np.zeros(meas.NUM_SAMPLES)
    saved_psi = np.zeros(meas.NUM_SAMPLES)

    prev_x_estimate = np.array([[0], [0], [0]])
    prev_P_estimate = 10 * np.identity(3)

    for idx in range(meas.NUM_SAMPLES):
        angular_vel = meas.get_gyro_measurements(idx)
        accel = meas.get_accelerometer_measurements(idx)
        euler_accel = meas.compute_euler_accel(accel)

        z = np.array([
            [euler_accel.phi],
            [euler_accel.theta],
        ])
        [new_x_estimate, new_P_estimate] = euler_ekf.run_kalman_filter(
            z, prev_x_estimate, prev_P_estimate, angular_vel, meas.DELTA_TIME
        )

        saved_timestamps[idx] = meas.DELTA_TIME * idx
        saved_phi[idx] = new_x_estimate[0] * 180 / math.pi
        saved_theta[idx] = new_x_estimate[1] * 180 / math.pi
        saved_psi[idx] = new_x_estimate[2] * 180 / math.pi

        prev_x_estimate = new_x_estimate
        prev_P_estimate = new_P_estimate

    return [saved_timestamps, saved_phi, saved_theta, saved_psi]


if __name__ == "__main__":
    [saved_timestamps, saved_phi, saved_theta, saved_psi] = run_ekf()
