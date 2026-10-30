import numpy as np
import math


def filterStep(x_pred, P_pred, v, H, R):
    if v is None or H is None or R is None:
        return x_pred, P_pred

    S = H @ P_pred @ H.T + R
    #Kovarijansa inovacije

    K = P_pred @ H.T @ np.linalg.inv(S)

    x_new = x_pred + K @ v

    x_new[2] = math.atan2(math.sin(x_new[2]), math.cos(x_new[2]))

    I = np.eye(P_pred.shape[0])
    P_new = (I - K @ H) @ P_pred

    return x_new, P_new