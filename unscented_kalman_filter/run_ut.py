#!/usr/bin/env python3

import numpy as np
import unscented_transformation as ut
import random
from plotly.subplots import make_subplots
import plotly.graph_objects as plotly_go

"""
xm: state variables
P: covariance
kappa: arbitrary constant
"""

N_SAMPLES = 100


def plot(saved_indices, saved_means_around_2, saved_means_around_4, saved_means_around_8):
    fig = make_subplots(
        rows=3,
        cols=1,
        x_title="Time (s)",
        subplot_titles=("centered around 2", "centered around 4", "centered around 8"),
        shared_yaxes=True,
    )
    fig.update_layout(title="Unscented Transformation")

    fig.append_trace(
        plotly_go.Scatter(
            x=saved_indices,
            y=saved_means_around_2,
            name="Estimated mean centered around 2",
            mode="markers",
        ),
        row=1,
        col=1,
    )
    fig.update_yaxes(title_text="stimated mean centered around 2", row=1, col=1)

    fig.append_trace(
        plotly_go.Scatter(
            x=saved_indices,
            y=saved_means_around_4,
            name="Estimated mean centered around 4",
            mode="markers",
        ),
        row=2,
        col=1,
    )
    fig.update_yaxes(title_text="stimated mean centered around 4", row=2, col=1)

    fig.append_trace(
        plotly_go.Scatter(
            x=saved_indices,
            y=saved_means_around_8,
            name="Estimated mean centered around 8",
            mode="markers",
        ),
        row=3,
        col=1,
    )
    fig.update_yaxes(title_text="stimated mean centered around 8", row=3, col=1)

    fig.write_html("unscented_transformation.html")


if __name__ == "__main__":
    saved_indices = np.zeros(N_SAMPLES)
    saved_means_around_2 = np.zeros(N_SAMPLES)
    saved_means_around_4 = np.zeros(N_SAMPLES)
    saved_means_around_8 = np.zeros(N_SAMPLES)

    xm = np.array([2, 4, 8])
    P = 10 * np.eye(3)

    for idx in range(N_SAMPLES):
        kappa = random.triangular(-3, 3, 0)

        [sigma_points, weights] = ut.compute_sigma_points(xm, P, kappa)
        [new_xm, new_P] = ut.unscented_transformation(sigma_points, weights)

        saved_indices[idx] = idx
        saved_means_around_2[idx] = new_xm[0]
        saved_means_around_4[idx] = new_xm[1]
        saved_means_around_8[idx] = new_xm[2]

    plot(
        saved_indices, saved_means_around_2, saved_means_around_4, saved_means_around_8
    )
