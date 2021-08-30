#!/usr/bin/env python3

import numpy as np
import math


class StateVariables:
    def __init__(self, x_position, velocity, altitude):
        self.x_position = x_position
        self.velocity = velocity
        self.altitude = altitude


def hx(estimated_state_variable):
    """
    Non-linear function to compute radar from state variable
    """
    return math.sqrt(
        math.pow(estimated_state_variable.x_position, 2)
        + math.pow(estimated_state_variable.altitude, 2)
    )


def compute_H(predicted_state_variable):
    """
    Compute the Jacobian matrix from the state-to-measurement function
    """
    H = np.matrix([0, 0, 0])
    denominator = math.sqrt(
        math.pow(predicted_state_variable.x_position, 2)
        + math.pow(predicted_state_variable.altitude, 2)
    )
    H[0] = predicted_state_variable.x_position / denominator
    H[1] = 0
    H[2] = predicted_state_variable.altitude / denominator


def run_radar_ekf(radar_measurement, num_samples, dt):

    return [0, 0, 0]
