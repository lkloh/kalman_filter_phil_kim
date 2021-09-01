#!/usr/bin/env python3
import random
import math


def generate_radar_measurement(dt, predicted_state):
    predicted_x_pos = predicted_state[0, 0]

    true_xvelocity = (
        100 + random.triangular(0, 100, 50) + 5 * random.triangular(0, 1, 0.5)
    )
    true_altitude = 1000 + 10 * random.triangular(0, 1, 0.5)

    estimated_x_position = predicted_x_pos + true_xvelocity * dt
    measurement_noise_of_radar = estimated_x_position * random.triangular(
        -0.1, 0.1, 0
    )
    return (
        math.sqrt(
            estimated_x_position * estimated_x_position + true_altitude * true_altitude
        )
        + measurement_noise_of_radar
    )
