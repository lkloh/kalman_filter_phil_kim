#!/usr/bin/env python3

import numpy as np
from numpy.linalg import inv
import math
from unscented_transformation import (compute_sigma_points, unscented_transformation)

def fx(state, dt):
    A = np.array(
        [
            [0, 1, 0],
            [0, 0, 0],
            [0, 0, 0],
        ]
    )
    discretized_A = np.eye(3) + dt * A
    return discretized_A @ state.transpose()


def hx(state):
    horizontal_dist = state[0]
    altitude = state[2]
    slant_dist = math.sqrt(math.pow(horizontal_dist, 2) + math.pow(altitude, 2))
    return slant_dist

class RadarUKF:
    def __init__(self, Q, R, kappa, dt, n_state_vars, n_measurements):
        self.Q = Q  # covariance of state transformation
        self.R = R  # Covariance of measurement noise
        self.kappa = kappa
        self.dt = dt
        self.n_state_vars = n_state_vars
        self.n_measurements = n_measurements

    def run_ukf(self, meas, prev_state, prev_covariance):
        # Compute Sigma points and weights
        [sigma, weights] = compute_sigma_points(prev_state, prev_covariance, self.kappa)
        print(sigma)