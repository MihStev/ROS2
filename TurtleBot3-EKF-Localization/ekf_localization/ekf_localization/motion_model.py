import numpy as np


def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def transitionFunction(x_prev, u, b):

    x = x_prev[0]
    y = x_prev[1]
    theta_prev = x_prev[2]

    delta_sl = u[0]
    delta_sr = u[1]

    delta_s = (delta_sl + delta_sr) / 2.0

    delta_theta = (delta_sr - delta_sl) / (2.0 * b)

    # jednacina (1): pomeraj se racuna u pravcu ugla na sredini koraka
    theta_mid = theta_prev + delta_theta / 2.0

    x_pred = np.array([
        x + delta_s * np.cos(theta_mid),
        y + delta_s * np.sin(theta_mid),
        normalize_angle(theta_prev + delta_theta)
    ])

    Fx = np.array([
        [1.0, 0.0, -delta_s * np.sin(theta_mid)],
        [0.0, 1.0,  delta_s * np.cos(theta_mid)],
        [0.0, 0.0, 1.0]
    ])

    c = np.cos(theta_mid)
    s = np.sin(theta_mid)

    Fu = np.array([
        [
            0.5 * c + (delta_s / (4.0 * b)) * s,
            0.5 * c - (delta_s / (4.0 * b)) * s
        ],
        [
            0.5 * s - (delta_s / (4.0 * b)) * c,
            0.5 * s + (delta_s / (4.0 * b)) * c
        ],
        [
            -1.0 / (2.0 * b),
             1.0 / (2.0 * b)
        ]
    ])

    return x_pred, Fx, Fu


def predictCovariance(P_prev, Fx, Fu,
                      delta_sl, delta_sr,
                      k):

    Q = np.array([
        [k * abs(delta_sl), 0.0],
        [0.0, k * abs(delta_sr)]
    ])

    P_pred = (
        Fx @ P_prev @ Fx.T +
        Fu @ Q @ Fu.T
    )

    return P_pred