#!/usr/bin/env python3
import numpy as np


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

    def run_kalman_filter(self, state, angular_rates):
        return np.array(
            [
                [None],
                [None],
                [None],
            ]
        )
