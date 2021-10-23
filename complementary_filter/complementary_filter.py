#!/usr/bin/env python3
import utils


def estimate_attitude_with_complementary_filter(
    accel_meas,
    gyro_meas,
    prev_angular_velocity_estimate,
    prev_attitude_estimate,
    prev_delta_attitude,
    dt,
):
    estimated_attitude_from_accel = utils.compute_euler_accel(accel_meas)

    change_in_prev_attitude_estimate = utils.body_frame_to_inertial_frame(
        gyro_meas, prev_attitude_estimate
    )

    current_phi_estimate = prev_attitude_estimate.phi + dt * (
        change_in_prev_attitude_estimate.phi - prev_angular_velocity_estimate.p
    )
    current_theta_estimate = prev_attitude_estimate.theta + dt * (
        change_in_prev_attitude_estimate.theta - prev_angular_velocity_estimate.q
    )
    current_psi_estimate = (
        prev_attitude_estimate.psi + dt * change_in_prev_attitude_estimate.psi
    )
    current_attitude_estimate = utils.EulerAngles(
        current_phi_estimate, current_theta_estimate, current_psi_estimate
    )

    current_angular_velocity_estimate = utils.AngularVelocities()
    current_delta_attitude = utils.EulerAngles()
    current_delta_attitude.phi = (
        current_phi_estimate - estimated_attitude_from_accel.phi
    )
    current_angular_velocity_estimate.p = (
        prev_angular_velocity_estimate.p
        + 0.1415 * current_delta_attitude.phi
        - 0.1414 * prev_delta_attitude.phi
    )
    current_delta_attitude.theta = (
        current_theta_estimate - estimated_attitude_from_accel.theta
    )
    current_angular_velocity_estimate.q = (
        prev_angular_velocity_estimate.q
        + 0.1415 * current_delta_attitude.theta
        - 0.1414 * prev_delta_attitude.theta
    )

    return [
        current_attitude_estimate,
        current_angular_velocity_estimate,
        current_delta_attitude,
    ]
