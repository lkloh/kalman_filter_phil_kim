#!/usr/bin/env python3

import numpy as np
from numpy.linalg import inv
import math


def compute_sigma_points(means, covariance, kappa):
    n_vars = len(means)
    means = means.reshape(n_vars, 1)

    sigma_points = np.zeros(shape=(n_vars, 2 * n_vars + 1))
    weights = np.zeros(2 * n_vars + 1)

    for i in range(n_vars):
        sigma_points[0, i] = means[i]
    weights[0] = kappa / (n_vars + kappa)

    # Where U^T @ U = (n_vars + kappa) * P
    U = np.linalg.cholesky((n_vars + kappa) * covariance)

    for i in range(n_vars):
        u_col = U[:, i].reshape(n_vars, 1)
        for v in range(n_vars):
            sigma_points[v, i + 1] = (means + u_col)[v]
        weights[i + 1] = 1 / (2 * (n_vars + kappa))
    for i in range(n_vars):
        u_col = U[:, i].reshape(n_vars, 1)
        for v in range(n_vars):
            sigma_points[v, i + 1] = (means - u_col)[v]
        weights[n_vars + i + 1] = 1 / (2 * (n_vars + kappa))

    return [sigma_points, weights]


def unscented_transformation(sigma_points, weights, noise_cov):
    [n_state_vars, n_sigma_pts] = sigma_points.shape

    new_xm = np.zeros(shape=(n_state_vars, 1))
    for i in range(n_sigma_pts):
        new_xm += weights[i] * sigma_points[:, i].reshape(n_state_vars, 1)

    new_P = np.zeros(shape=(n_state_vars, n_state_vars))
    for i in range(n_sigma_pts):
        temp = sigma_points[:, i].reshape(n_state_vars, 1) - new_xm
        new_P += weights[i] * (temp @ temp.transpose())

    return [new_xm, new_P + noise_cov]
