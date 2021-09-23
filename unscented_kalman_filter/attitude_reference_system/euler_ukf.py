#!/usr/bin/env python3
import math
import numpy as np
from ut import compute_sigma_points, unscented_transformation


def fx(estimated_state, angular_rates, dt):
    phi = estimated_state[0]
    theta = estimated_state[1]

    p = angular_rates.p
    q = angular_rates.q
    r = angular_rates.r

    xdot = np.zeros(3)
    xdot[0] = (
        p + q * math.sin(phi) * math.tan(theta) + r * math.cos(phi) * math.tan(theta)
    )
    xdot[1] = q * math.cos(phi) - r * math.sin(phi)
    xdot[2] = q * math.sin(phi) / math.cos(theta) + r * math.cos(phi) / math.cos(theta)

    return estimated_state + xdot * dt


class EulerUKF:
    def __init__(
        self,
        state_transition_noise_cov,
        measurement_noise_covariance,
        kappa,
        dt,
        n_state_vars,
        n_measurements,
    ):
        self.state_transition_noise_cov = state_transition_noise_cov
        self.measurement_noise_covariance = measurement_noise_covariance
        self.kappa = kappa
        self.dt = dt
        self.n_state_vars = n_state_vars
        self.n_measurements = n_measurements

    def run_kalman_filter(
        self, meas, angular_rates_from_gyro, prev_state, prev_error_covariance
    ):
        # Compute sigma points and weights
        [sigma, weights] = compute_sigma_points(
            prev_state, prev_error_covariance, self.kappa
        )

        # Predict state and error covariance
        f_sigma = np.zeros(shape=(self.n_state_vars, 2 * self.n_state_vars + 1))
        for col in range(2 * self.n_state_vars + 1):
            f_sigma[:, col] = fx(sigma[:, col], angular_rates_from_gyro, self.dt)

        return np.array(
            [
                [None],
                [None],
                [None],
            ]
        )
