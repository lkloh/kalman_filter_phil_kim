#!/usr/bin/env python3

import numpy as np
from get_radar import generate_radar_measurement
from ukf import RadarUKF

Q = 0.01 * np.eye(3)  # Covariance of of state transition noise
R = 100  # Covariance of measurement noise
KAPPA = 0

N_STATE_VARS = 3  # state vars are: x-position,
N_MEASUREMENT_VARS = 1
DELTA_TIME = 0.05
N_RADAR_SAMPLES = 10


def run_ukf():
    saved_timestamps = np.zeros(N_RADAR_SAMPLES)
    saved_position = np.zeros(N_RADAR_SAMPLES)
    saved_velocity = np.zeros(N_RADAR_SAMPLES)
    saved_altitude = np.zeros(N_RADAR_SAMPLES)
    saved_measurements = np.zeros(N_RADAR_SAMPLES)

    prev_state = np.array([0, 90, 1100])
    prev_error_covariance = 100 * np.eye(3)

    radar_ukf = RadarUKF(Q, R, KAPPA, DELTA_TIME, N_STATE_VARS, N_MEASUREMENT_VARS)

    for i in range(N_RADAR_SAMPLES):
        radar_meas = generate_radar_measurement(prev_state, DELTA_TIME)
        [current_state, current_error_covariance] = radar_ukf.run_ukf(
            radar_meas, prev_state, prev_error_covariance
        )

        saved_timestamps[i] = i * DELTA_TIME
        saved_measurements[i] = radar_meas
        saved_position[i] = current_state[0]
        saved_velocity[i] = current_state[1]
        saved_altitude[i] = current_state[2]

        prev_state = current_state
        prev_error_covariance = current_error_covariance

    return [
        saved_timestamps,
        saved_position,
        saved_velocity,
        saved_altitude,
        saved_measurements,
    ]


if __name__ == "__main__":
    [
        saved_timestamps,
        saved_position,
        saved_velocity,
        saved_altitude,
        saved_measurements,
    ] = run_ukf()
