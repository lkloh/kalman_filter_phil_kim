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
    weights = np.zeros(shape=(2 * n_vars + 1, 1))

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

    print(sigma_points)
    print(weights)
    print(U)


if __name__ == "__main__":
    xm = np.array([1, 2, 3])
    P = 1.2 * np.eye(3)
    compute_sigma_points(xm, P, 2)
