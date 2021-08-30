#!/usr/bin/env python3

import numpy as np
import math
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
from scipy.io import loadmat
from get_radar import generate_radar_measurement
from radar_ekf import run_radar_ekf

NUM_SAMPLES = 100
DELTA_TIME = 0.5

def plot_results(saved_timestamps, saved_estimated_positions, saved_estimated_velocity, saved_estimated_altitude):
    pass

def run_ekf():
    saved_timestamps = np.zeros(NUM_SAMPLES)
    saved_estimated_positions = np.zeros(NUM_SAMPLES)
    saved_estimated_velocity = np.zeros(NUM_SAMPLES)
    saved_estimated_altitude = np.zeros(NUM_SAMPLES)

    predicted_x_position = 0
    for i in range(NUM_SAMPLES):
        radar_measurement = generate_radar_measurement(DELTA_TIME, predicted_x_position)
        [estimated_pos, estimated_vel, estimated_alt] = run_radar_ekf(
            radar_measurement, NUM_SAMPLES, DELTA_TIME
        )

        saved_timestamps[i] = i * DELTA_TIME
        saved_estimated_positions[i] = estimated_pos
        saved_estimated_velocity[i] = estimated_vel
        saved_estimated_altitude[i] = estimated_alt

    return [
        saved_estimated_positions,
        saved_estimated_velocity,
        saved_estimated_altitude,
    ]


if __name__ == "__main__":
    [
        saved_timestamps,
        saved_estimated_positions,
        saved_estimated_velocity,
        saved_estimated_altitude,
    ] = run_ekf()

    plot_results(saved_timestamps, saved_estimated_positions, saved_estimated_velocity, saved_estimated_altitude)
