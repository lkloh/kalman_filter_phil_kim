#!/usr/bin/env python3
import random
import math


def generate_radar_measurement(dt, predicted_xposition):
    if predicted_xposition is None:
        predicted_xposition = 0

    true_xvelocity = (
        100 + random.triangular(0, 100, 50) + 5 * random.triangular(0, 1, 0.5)
    )
    true_altitude = 1000 + 10 * random.triangular(0, 1, 0.5)

    estimated_xposition = predicted_xposition + true_xvelocity * dt
    measurement_noise_of_radar = estimated_xposition * random.triangular(0, 0.05, 0.025)
    measured_radar = (
        math.sqrt(
            estimated_xposition * estimated_xposition + true_altitude * true_altitude
        )
        + measurement_noise_of_radar
    )

    predicted_xposition = estimated_xposition
    return measured_radar
