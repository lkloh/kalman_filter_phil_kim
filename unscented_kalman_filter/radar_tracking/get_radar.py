#!/usr/bin/env python3
import random
import math


def generate_radar_measurement(predicted_state, dt):
    predicted_x_pos = predicted_state[0]

    true_x_velocity = 100 + 5 * random.triangular(-1, 1, 0)
    true_altitude = 1000 + 10 * random.triangular(-1, 1, 0)

    estimated_x_position = predicted_x_pos + true_x_velocity * dt
    measurement_noise_of_radar = 0.05 * random.triangular(-1, 1, 0)
    return math.sqrt(math.pow(estimated_x_position, 2) + math.pow(true_altitude,2)) + measurement_noise_of_radar
