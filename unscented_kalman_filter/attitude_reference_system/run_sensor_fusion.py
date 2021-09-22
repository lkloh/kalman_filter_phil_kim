#!/usr/bin/env python3

import numpy as np
import math
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
import retrieve_data as ats
from euler_ukf import EulerUKF

STATE_TRANSITION_NOISE_COVARIANCE = np.array(
    [[0.0001, 0, 0], [0, 0.0001, 0], [0, 0, 1]]
)
MEASUREMENT_NOISE_COVARIANCE = np.array([[6, 0], [0, 12]])

N_STATE_VARS = 3
N_MEAS = 2
KAPPA = 0


def run_ukf():
    prev_state = np.array([[0], [0], [0]])
    prev_error_covariance = np.eye(3)

    for i in range(ats.NUM_GYRO_MEAS):
        gyro_meas = ats.get_gyro_measurements(i)
        accel_meas = ats.get_accelerometer_measurements(i)

        accel_meas = ats.compute_euler_accel(accel_meas)
        meas = np.array([[accel_meas.phi], [accel_meas.theta]])
        ukf = EulerUKF(
            STATE_TRANSITION_NOISE_COVARIANCE,
            MEASUREMENT_NOISE_COVARIANCE,
            KAPPA,
            ats.DELTA_TIME,
            N_STATE_VARS,
            N_MEAS,
        )
        estimated_state = ukf.run_kalman_filter(meas, gyro_meas)


if __name__ == "__main__":
    if ats.NUM_GYRO_MEAS != ats.NUM_ACCEL_MEAS:
        print(
            "Number of gyroscope measurements not equal to number of accelerometer measurements, exiting."
        )
        exit(1)

    run_ukf()
