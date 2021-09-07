#!/usr/bin/env python3

import numpy as np
import math
from numpy.linalg import inv

H = np.array([[1, 0, 0], [0, 1, 0]])

Q = np.array([[0.0001, 0, 0], [0, 0.0001, 0], [0, 0, 0.1]])

R = 6 * np.identity(2)


def A_jacobian(estimated_x, angular_vel, dt):
    A = np.zeros((3, 3))

    phi = estimated_x[0]
    theta = estimated_x[1]

    p = angular_vel.p
    q = angular_vel.q
    r = angular_vel.r

    A[0, 0] = p * math.cos(phi) * math.tan(theta) - r * math.sin(phi) * math.tan(theta)
    A[0, 1] = q * math.sin(phi) / math.pow(math.cos(theta), 2) + r * math.cos(
        phi
    ) / math.pow(math.cos(theta), 2)
    A[0, 2] = 0

    A[1, 0] = -1 * q * math.sin(phi) - r * math.cos(phi)
    A[1, 1] = 0
    A[1, 2] = 0

    A[2, 0] = q * math.cos(phi) / math.cos(theta) - r * math.sin(phi) / math.cos(theta)
    A[2, 1] = q * math.sin(phi) * math.tan(theta) / math.cos(theta) + r * math.cos(
        phi
    ) * math.tan(theta) / math.cos(theta)
    A[2, 2] = 0

    return np.identity(3) + A * dt


def compute_predicted_state(prev_x_estimate, angular_vel, dt):
    phi = prev_x_estimate[0]
    theta = prev_x_estimate[1]

    xdot = np.array(
        [
            [
                angular_vel.p
                + angular_vel.q * math.sin(phi) * math.tan(theta)
                + angular_vel.r * math.cos(phi) * math.tan(theta)
            ],
            [angular_vel.q * math.cos(phi) - angular_vel.r * math.sin(phi)],
            [
                angular_vel.q * math.sin(phi) / math.cos(theta)
                + angular_vel.r * math.cos(phi) / math.cos(theta)
            ],
        ]
    )
    return prev_x_estimate + xdot * dt


def run_kalman_filter(z, prev_x_estimate, prev_P_estimate, angular_vel, dt):
    A = A_jacobian(prev_x_estimate, angular_vel, dt)

    # Step I: Predict state
    x_predict = compute_predicted_state(prev_x_estimate, angular_vel, dt)
    # Step I: Predict error covariance
    P_predict = A * prev_P_estimate * A.transpose() + Q

    # Step II: Compute Kalman Gain
    K = P_predict * H.transpose() * inv(H * P_predict * H.transpose() + R)

    # Step III: Compute estimate
    x_estimate = x_predict + K * (z - H * x_predict)

    # Step IV: Compute error covariance
    P_estimate = P_predict - K * H * P_predict

    return [x_estimate, P_estimate]
