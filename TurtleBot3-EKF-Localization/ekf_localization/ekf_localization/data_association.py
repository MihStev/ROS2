import numpy as np
import math

from ekf_localization.measurement_model import measurementFunction

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def associateMeasurements(x_hat, P_hat, Z, R_list, M, g):
    """
    x_hat  = predikcija pozicije
    P_hat  = apriorna kovarijansa 3x3 (zadatak 1)
    Z      = opservacije iz lidara, lista linija [alpha, r]
    R_list = lista kovarijansi merenja, svaka 2x2
    M      = mapa, lista linija [alpha, r]
    g      = prag validacije

    """

    accepted_v = []
    accepted_H = []
    accepted_R = []

    for j in range(len(Z)):

        z = Z[j]
        R = R_list[j]

        best_d = None
        best_v = None
        best_H = None
        best_R = None

        for i in range(len(M)):

            m = M[i]

            z_hat, H = measurementFunction(x_hat, m)

            # inovacija v = z - z_hat
            v = z - z_hat

            # ugao mora da ostane u opsegu [-pi, pi]
            v[0] = normalize_angle(v[0])

            # kovarijansa inovacije
            S = H @ P_hat @ H.T + R

            # Mahalanobisova distanca
            d = v.T @ np.linalg.inv(S) @ v

            if best_d is None or d < best_d:
                best_d = d
                best_v = v
                best_H = H
                best_R = R

        # prihvatamo samo ako je distanca manja od g^2
        if best_d is not None and best_d < g**2:
            accepted_v.append(best_v)
            accepted_H.append(best_H)
            accepted_R.append(best_R)


    if len(accepted_v) == 0:
        return None, None, None

    v_hat = np.concatenate(accepted_v)

    H_hat = np.vstack(accepted_H)

    n = len(accepted_R)
    R_hat = np.zeros((2 * n, 2 * n))

    for i in range(n):
        R_hat[2*i:2*i+2, 2*i:2*i+2] = accepted_R[i]

    return v_hat, H_hat, R_hat