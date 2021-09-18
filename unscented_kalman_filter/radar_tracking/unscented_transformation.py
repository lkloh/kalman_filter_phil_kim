#!/usr/bin/env python3
import numpy as np
from numpy.linalg import inv
import math

"""
xm: state variables
P: covariance
kappa: arbitrary constant
"""

def compute_sigma_points(xm, P, kappa):
    n_vars = len(xm)
    sigma_points = np.zeros(shape=(2 * n_vars + 1, n_vars))
    weights = np.zeros(2 * n_vars + 1)

    for i in range(n_vars):
        sigma_points[i, 0] = xm[i]
    weights[0] = kappa / (n_vars + kappa)

    # Where U^T @ U = (n_vars + kappa) * P
    U = np.linalg.cholesky((n_vars + kappa) * P)

    for i in range(n_vars):
        sigma_points[i + 1, :] = xm + U[i, :]
        weights[i + 1] = 1 / (2 * (n_vars + kappa))
    for i in range(n_vars):
        sigma_points[n_vars + i + 1, :] = xm - U[i, :]
        weights[n_vars + i + 1] = 1 / (2 * (n_vars + kappa))

    return [sigma_points, weights]


def unscented_transformation(sigma_points, weights, noise_covariance):
    [n_sigma_pts, n_state_vars] = sigma_points.shape

    new_xm = np.zeros(n_state_vars)
    for i in range(n_sigma_pts):
        new_xm += weights[i] * sigma_points[i, :]

    new_P = np.zeros(shape=(n_state_vars, n_state_vars))
    for i in range(n_sigma_pts):
        temp = sigma_points[i, :] - new_xm
        new_P += weights[i] * (temp @ temp.transpose())

    return [new_xm, new_P + noise_covariance]
