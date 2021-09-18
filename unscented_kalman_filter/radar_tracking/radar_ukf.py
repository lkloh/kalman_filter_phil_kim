#!/usr/bin/env python3

import numpy as np
from numpy.linalg import inv
import math
from unscented_transformation import compute_sigma_points, unscented_transformation


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

    def run_ukf(self, z, prev_state, prev_P):
        # (1) Compute Sigma points and weights
        [sigma, weights] = compute_sigma_points(prev_state, prev_P, self.kappa)

        # (2) Predict state & error covariance
        f_sigma = np.zeros(shape=(2 * self.n_state_vars + 1, self.n_state_vars))
        for idx in range(2 * self.n_state_vars + 1):
            f_sigma[idx, :] = fx(sigma[idx, :], self.dt)
        [predicted_state, predicted_state_covariance] = unscented_transformation(
            f_sigma, weights
        )

        # (3) predict measurement & covariance
        h_sigma = np.zeros(shape=(2 * self.n_state_vars + 1, self.n_measurements))
        for idx in range(2 * self.n_state_vars + 1):
            h_sigma[idx, :] = hx(prev_state)
        [predicted_meas, predicted_meas_covariance] = unscented_transformation(
            h_sigma, weights
        )

        return np.zeros(3)
