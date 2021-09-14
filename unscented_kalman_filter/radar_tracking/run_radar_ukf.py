#!/usr/bin/env python3

import numpy as np
from get_radar import generate_radar_measurement
from radar_ukf import RadarUKF

Q = 0.01 * np.eye(3)
R = 100
KAPPA = 0

N_STATE_VARS = 3
N_MEASUREMENT_VARS = 1
DELTA_TIME = 0.01
N_RADAR_SAMPLES = 400

def run_ukf():
    saved_timestamps = np.zeros(N_RADAR_SAMPLES)
    saved_position = np.zeros(N_RADAR_SAMPLES)
    saved_velocity = np.zeros(N_RADAR_SAMPLES)
    saved_altitude = np.zeros(N_RADAR_SAMPLES)

    prev_state = np.array([
        [0],
        [90],
        [1100]
    ])
    prev_covariance = 100 * np.eye(3)

    for i in range(N_RADAR_SAMPLES):
        z = generate_radar_measurement(prev_state, DELTA_TIME)
        ukf = RadarUKF(Q, R, KAPPA, DELTA_TIME, N_STATE_VARS, N_MEASUREMENT_VARS)
        current_state = ukf.run_ukf(z)

        saved_timestamps[i] = i * DELTA_TIME

        prev_state = current_state

    return [saved_timestamps, saved_position,  saved_velocity, saved_altitude]

if __name__ == "__main__":
    [saved_timestamps, saved_position,  saved_velocity, saved_altitude] = run_ukf()
