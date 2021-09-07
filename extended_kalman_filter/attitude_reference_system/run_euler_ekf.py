#!/usr/bin/env python3

import numpy as np
import math
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
import retrieve_data as meas
import euler_ekf


def plot_estimates(saved_timestamps, saved_phi, saved_theta, saved_psi):
    fig = make_subplots(
        rows=3,
        cols=1,
        x_title="Time (s)",
    )
    fig.update_layout(title="Extended Kalman Filter for Attitude Estimation")

    fig.append_trace(
        plotly_go.Scatter(
            x=saved_timestamps,
            y=saved_phi,
            name="Estimated Roll Angle [deg]",
            mode="lines+markers",
        ),
        row=1,
        col=1,
    )
    fig.update_yaxes(title_text="Roll Angle [deg]", row=1, col=1)

    fig.append_trace(
        plotly_go.Scatter(
            x=saved_timestamps,
            y=saved_theta,
            name="Estimated Pitch Angle [deg]",
            mode="lines+markers",
        ),
        row=2,
        col=1,
    )
    fig.update_yaxes(title_text="Pitch Angle [deg]", row=2, col=1)

    fig.append_trace(
        plotly_go.Scatter(
            x=saved_timestamps,
            y=saved_psi,
            name="Estimated Yaw Angle [deg]",
            mode="lines+markers",
        ),
        row=3,
        col=1,
    )
    fig.update_yaxes(title_text="Yaw Angle [deg]", row=3, col=1)

    fig.write_html("chapter14.5_ekf_for_attitude.html")


def run_ekf():
    if meas.NUM_GYRO_MEAS != meas.NUM_ACCEL_MEAS:
        print(
            "Number of gyroscope measurements is %i which is not equal to number of accelerometer measurements",
            meas.NUM_GYRO_MEAS,
            meas.NUM_ACCEL_MEAS,
        )
        quit()
    NUM_SAMPLES = meas.NUM_GYRO_MEAS

    saved_timestamps = np.zeros(NUM_SAMPLES)
    saved_phi = np.zeros(NUM_SAMPLES)
    saved_theta = np.zeros(NUM_SAMPLES)
    saved_psi = np.zeros(NUM_SAMPLES)

    prev_x_estimate = np.array([[0], [0], [0]])
    prev_P_estimate = 10 * np.identity(3)

    for idx in range(NUM_SAMPLES):
        angular_vel = meas.get_gyro_measurements(idx)
        accel = meas.get_accelerometer_measurements(idx)
        euler_accel = meas.compute_euler_accel(accel)

        z = np.array(
            [
                [euler_accel.phi],
                [euler_accel.theta],
            ]
        )
        [new_x_estimate, new_P_estimate] = euler_ekf.run_kalman_filter(
            z, prev_x_estimate, prev_P_estimate, angular_vel, meas.DELTA_TIME
        )

        saved_timestamps[idx] = meas.DELTA_TIME * idx
        saved_phi[idx] = new_x_estimate[0] * 180 / math.pi
        saved_theta[idx] = new_x_estimate[1] * 180 / math.pi
        saved_psi[idx] = new_x_estimate[2] * 180 / math.pi

        prev_x_estimate = new_x_estimate
        prev_P_estimate = new_P_estimate

    return [saved_timestamps, saved_phi, saved_theta, saved_psi]


if __name__ == "__main__":
    [saved_timestamps, saved_phi, saved_theta, saved_psi] = run_ekf()
    plot_estimates(saved_timestamps, saved_phi, saved_theta, saved_psi)
