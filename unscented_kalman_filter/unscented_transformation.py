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
    sigma_points = np.zeros(shape=(n_vars, 2 * n_vars + 1))
    weights = np.zeros(shape=(n_vars, 1))

    for i in range(n_vars):
        sigma_points[i, 0] = xm[i]
    weights[0] = kappa / (n_vars + kappa)

    print(sigma_points)
    print(weights)


if __name__ == "__main__":
    xm = np.array([[1], [2], [3]])
    P = np.array([[1.3], [1.2], [0.9]])
    compute_sigma_points(xm, P, 2)
