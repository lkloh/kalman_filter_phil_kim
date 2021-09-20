#!/usr/bin/env python3

import numpy as np
import unscented_transformation as ut

"""
xm: state variables
P: covariance
kappa: arbitrary constant
"""

KAPPA = 0

if __name__ == "__main__":
    xm = np.array([2.1, 2.2, 1.9])
    P = 10 * np.eye(3)
    [sigma_points, weights] = ut.compute_sigma_points(xm, P, KAPPA)
    [new_xm, new_P] = ut.unscented_transformation(sigma_points, weights)

    print(new_xm)
