import numpy as np


def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def measurementFunction(x_pred, m):

    x = x_pred[0]
    y = x_pred[1]
    theta = x_pred[2]

    alpha = m[0]
    r = m[1]

    alpha_pred = normalize_angle(alpha - theta)

    r_pred = r - (
        x * np.cos(alpha) +
        y * np.sin(alpha)
    )

    z_pred = np.array([
        alpha_pred,
        r_pred
    ])

    Hx = np.array([
        [ 0.0, 0.0, -1.0],
        [-np.cos(alpha), -np.sin(alpha), 0.0]
    ])

    return z_pred, Hx