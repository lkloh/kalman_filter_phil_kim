#!/usr/bin/env python3

import numpy as np
import random
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go
import retrieve_data as data
from complementary_filter import (
    estimate_attitude_with_complementary_filter,
)
import utils


def plot_results(saved_timestamps, saved_phi, saved_theta):
    fig = make_subplots(
        rows=2,
        cols=1,
        x_title="Time (s)",
    )
    fig.update_layout(title="Complementary Filter for Attitude Estimation")

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

    fig.write_html("chapter17.3_complementary_filter_for_attitude.html")


def run_filter():
    if data.NUM_GYRO_MEAS != data.NUM_ACCEL_MEAS:
        print(
            "Number of gyroscope measurements ({}) is not equal to number of accelerometer measurements ({})".format(
                data.NUM_GYRO_MEAS, data.NUM_ACCEL_MEAS
            )
        )
        exit(1)

    saved_timestamps = np.zeros(data.NUM_GYRO_MEAS)
    saved_phi = np.zeros(data.NUM_GYRO_MEAS)
    saved_theta = np.zeros(data.NUM_GYRO_MEAS)

    prev_angular_velocity_estimate = utils.AngularVelocities(0, 0, 0)
    prev_attitude_estimate = utils.EulerAngles(0, 0, 0)
    prev_delta_attitude = utils.EulerAngles(0, 0, 0)

    for i in range(data.NUM_GYRO_MEAS):
        accel_meas = data.get_accelerometer_measurements(i)
        gyro_meas = data.get_gyro_measurements(i)

        [
            current_attitude_estimate,
            current_angular_velocity_estimate,
            current_delta_attitude,
        ] = estimate_attitude_with_complementary_filter(
            accel_meas,
            gyro_meas,
            prev_angular_velocity_estimate,
            prev_attitude_estimate,
            prev_delta_attitude,
            data.DELTA_TIME,
        )

        saved_timestamps[i] = i * data.DELTA_TIME
        saved_phi[i] = current_attitude_estimate.phi
        saved_theta[i] = current_attitude_estimate.theta

        # update iteration
        prev_angular_velocity_estimate = current_angular_velocity_estimate
        prev_attitude_estimate = current_attitude_estimate
        prev_delta_attitude = current_delta_attitude

    return [saved_timestamps, saved_phi, saved_theta]


if __name__ == "__main__":
    [saved_timestamps, saved_phi, saved_theta] = run_filter()
    plot_results(saved_timestamps, saved_phi, saved_theta)
