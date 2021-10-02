#!/usr/bin/env python3
import utils


def estimate_attitude_with_complementary_filter(accel_meas, gyro_meas, prev_angular_velocity_estimate, prev_attitude_estimate, dt):
    estimated_attitude_from_accel = utils.compute_euler_accel(accel_meas)

    change_in_prev_attitude_estimate = utils.body_frame_to_inertial_frame(gyro_meas, prev_attitude_estimate)

    current_phi_estimate = prev_attitude_estimate.phi + dt * (change_in_prev_attitude_estimate.phi - prev_angular_velocity_estimate.p)
    current_theta_estimate = prev_attitude_estimate.theta + dt * (change_in_prev_attitude_estimate.theta - prev_angular_velocity_estimate.q)
    current_psi_estimate = prev_attitude_estimate.psi + dt * change_in_prev_attitude_estimate.psi
    current_attitude_estimate = utils.EulerAngles(current_phi_estimate, current_theta_estimate, current_psi_estimate)




    return utils.EulerAngles()
