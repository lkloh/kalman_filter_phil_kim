#!/usr/bin/env python3

import numpy as np
import math
from numpy.linalg import inv


class StateVariables:
    def __init__(self, x_position, velocity, altitude):
        self.x_position = x_position
        self.velocity = velocity
        self.altitude = altitude


class RadarEKF:
    def __init__(self, A, Q, R):
        self.A = A
        self.Q = Q
        self.R = R

    def hx(self, estimated_state):
        """
        Non-linear function to compute radar from state variable
        """
        estimated_x_pos = estimated_state[0,0]
        estimated_alt = estimated_state[2,0]
        return math.sqrt(math.pow(estimated_x_pos, 2) + math.pow(estimated_alt, 2))


    def compute_H_jacobian(self, predicted_state):
        """
        Compute the Jacobian matrix from the state-to-measurement function
        """
        predicted_x_pos = predicted_state[0,0]
        predicted_alt = predicted_state[2,0]
        denominator = math.sqrt(
            math.pow(predicted_x_pos, 2) + math.pow(predicted_alt, 2)
        )
        return np.matrix([predicted_x_pos / denominator, 0, predicted_alt / denominator])


    def run_kalman_filter(self, z, prev_x_estimate, prev_P_estimate):
        H_jacob = self.compute_H_jacobian(prev_x_estimate)

        # Step I: Predict state
        x_predict = self.A * prev_x_estimate
        # Step I: Predict error covariance
        P_predict = self.A * prev_P_estimate * self.A.transpose() + self.Q

        # Step II: Compute Kalman Gain
        K = (
            P_predict
            * H_jacob.transpose()
            * inv(H_jacob * P_predict * H_jacob.transpose() + self.R)
        )

        # Step III: Compute estimate
        x_estimate = x_predict + K * (z - self.hx(x_predict))

        # Step IV: Compute error covariance
        P_estimate = P_predict - K * H_jacob * P_predict

        return [x_estimate, P_estimate]
