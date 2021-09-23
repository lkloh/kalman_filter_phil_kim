#!/usr/bin/env python3
import math
import numpy as np
from ut import compute_sigma_points, unscented_transformation


def hx(state):
    phi = state[0]
    theta = state[1]

    return np.array([phi, theta])


def fx(state, angular_rates, dt):
    phi = state[0]
    theta = state[1]

    p = angular_rates.p
    q = angular_rates.q
    r = angular_rates.r

    xdot = np.zeros(3)
    xdot[0] = (
        p + q * math.sin(phi) * math.tan(theta) + r * math.cos(phi) * math.tan(theta)
    )
    xdot[1] = q * math.cos(phi) - r * math.sin(phi)
    xdot[2] = q * math.sin(phi) / math.cos(theta) + r * math.cos(phi) / math.cos(theta)

    return state + xdot * dt


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

        # Predict state and associated error covariance
        f_sigma = np.zeros(shape=(self.n_state_vars, 2 * self.n_state_vars + 1))
        for col in range(2 * self.n_state_vars + 1):
            f_sigma[:, col] = fx(sigma[:, col], angular_rates_from_gyro, self.dt)
        [predicted_state, predicted_error_covariance] = unscented_transformation(
            f_sigma, weights, self.state_transition_noise_cov
        )

        # Predict measurement and associated error covariance
        h_sigma = np.zeros(shape=(self.n_measurements, 2 * self.n_state_vars + 1))
        for col in range(2 * self.n_state_vars + 1):
            h_sigma[:, col] = hx(f_sigma[:, col])
        [predicted_meas, predicted_meas_error_covariance] = unscented_transformation(
            h_sigma, weights, self.measurement_noise_covariance
        )

        # Compute Kalman Gain
        state_meas_covariance = np.zeros(shape=(self.n_state_vars, self.n_measurements))
        for col in range(2 * self.n_state_vars + 1):
            state_tmp = f_sigma[:, col].reshape(self.n_state_vars, 1) - predicted_state
            meas_tmp = h_sigma[:, col].reshape(self.n_measurements, 1) - predicted_meas
            state_meas_covariance += weights[col] * (state_tmp @ meas_tmp.transpose())

        return np.array(
            [
                [None],
                [None],
                [None],
            ]
        )
