#!/usr/bin/env python3

import numpy as np
from numpy.linalg import inv
import math
from unscented_transformation import (compute_sigma_points, run_unscented_transformation)

def fx(x, dt):
    A = np.array([
        [0, 1, 0],
        [0, 0, 0],
        [0, 0, 0],
    ])
    discretized_A = np.eye(3) + dt * A
    return (discretized_A @ x)


def hx(x):
    horizontal_dist = x[10]
    altitude = x[2]
    slant_dist = math.sqrt(math.pow(horizontal_dist, 2) + math.pow(altitude, 2))
    return slant_dist

class RadarUKF:
    def __init__(self, Q, R, kappa, dt, n_state_vars, n_measurements):
        self.Q = Q # covariance of state transformation
        self.R = R # Covariance of measurement noise
        self.kappa = kappa
        self.dt = dt
        self.n_state_vars = n_state_vars
        self.n_measurements = n_measurements


    def run_ukf(self, z, prev_x, prev_P):
        [sigma, weights] = compute_sigma_points(prev_x, prev_P, self.kappa)
        f_sigma = np.zeros(shape=(2 * self.n_state_vars + 1, self.n_state_vars))
        for idx in range(2 * self.n_state_vars + 1):
            f_sigma[idx,:] = fx(sigma[idx,:].transpose(), self.dt)



        return np.zeros(3)


