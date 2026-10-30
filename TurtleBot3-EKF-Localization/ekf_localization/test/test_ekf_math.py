"""Offline checks of the EKF building blocks (no simulator needed)."""
import math

import numpy as np

from ekf_localization.data_association import associateMeasurements
from ekf_localization.ekf_update import filterStep
from ekf_localization.measurement_model import measurementFunction
from ekf_localization.motion_model import predictCovariance, transitionFunction
from ekf_localization.split_and_merge import split_and_merge_iterative

B = 0.08
RNG = np.random.default_rng(0)


def numerical_jacobian(f, x0, eps=1e-6):
    y0 = f(x0)
    J = np.zeros((len(y0), len(x0)))
    for i in range(len(x0)):
        d = np.zeros(len(x0))
        d[i] = eps
        J[:, i] = (f(x0 + d) - f(x0 - d)) / (2 * eps)
    return J


def raycast_box(pose, xlim=(-1.5, 1.5), ylim=(-1.5, 1.5), n=360, noise=0.0):
    """Simulate a 360-beam lidar inside an axis-aligned rectangular room."""
    x, y, th = pose
    pts = []
    for k in range(n):
        a = -math.pi + k * 2 * math.pi / n
        c, s = math.cos(th + a), math.sin(th + a)
        ts = []
        if abs(c) > 1e-9:
            ts += [(xlim[0] - x) / c, (xlim[1] - x) / c]
        if abs(s) > 1e-9:
            ts += [(ylim[0] - y) / s, (ylim[1] - y) / s]
        t = min(t for t in ts if t > 0) + RNG.normal(0, noise)
        pts.append((t * math.cos(a), t * math.sin(a)))
    return pts


BOX_MAP = [np.array([0.0, 1.5]), np.array([math.pi, 1.5]),
           np.array([math.pi / 2, 1.5]), np.array([-math.pi / 2, 1.5])]


def test_motion_jacobians_match_numerical():
    for _ in range(100):
        x = RNG.uniform([-2, -2, -3], [2, 2, 3])
        u = RNG.uniform(-0.05, 0.05, 2)
        _, Fx, Fu = transitionFunction(x, u, B)
        assert np.allclose(Fx, numerical_jacobian(
            lambda xx: transitionFunction(xx, u, B)[0], x), atol=1e-7)
        assert np.allclose(Fu, numerical_jacobian(
            lambda uu: transitionFunction(x, uu, B)[0], u), atol=1e-7)


def test_measurement_jacobian_matches_numerical():
    for _ in range(100):
        x = RNG.uniform([-2, -2, -3], [2, 2, 3])
        m = RNG.uniform([-3, 0.2], [3, 3])
        _, H = measurementFunction(x, m)
        assert np.allclose(H, numerical_jacobian(
            lambda xx: measurementFunction(xx, m)[0], x), atol=1e-7)


def test_pure_rotation():
    x, _, _ = transitionFunction(np.zeros(3), np.array([-0.01, 0.01]), B)
    assert np.allclose(x, [0.0, 0.0, 0.02 / (2 * B)])


def test_covariance_grows_without_measurements():
    P = np.eye(3) * 1e-3
    x = np.zeros(3)
    for _ in range(50):
        u = np.array([0.01, 0.012])
        x, Fx, Fu = transitionFunction(x, u, B)
        P_new = predictCovariance(P, Fx, Fu, u[0], u[1], 0.01)
        assert np.trace(P_new) > np.trace(P)
        P = P_new


def test_split_and_merge_finds_room_walls():
    pose = (0.3, -0.2, 0.4)
    lines = split_and_merge_iterative(raycast_box(pose, noise=0.005), 0.08)
    expected = [measurementFunction(np.array(pose), m)[0] for m in BOX_MAP]
    for alpha, r in expected:
        assert any(abs(math.remainder(theta - alpha, 2 * math.pi)) < 0.05 and abs(rho - r) < 0.05
                   for rho, theta, _, _ in lines)


def test_association_rejects_outlier():
    x = np.array([0.2, 0.1, 0.0])
    P = np.eye(3) * 1e-3
    Z = [measurementFunction(x, m)[0] for m in BOX_MAP] + [np.array([0.7, 0.3])]
    R = [np.diag([0.05 ** 2, 0.02 ** 2])] * len(Z)
    v, H, R_hat = associateMeasurements(x, P, Z, R, BOX_MAP, 3.0)
    assert v.shape == (8,) and H.shape == (8, 3) and R_hat.shape == (8, 8)
    assert np.allclose(v, 0.0, atol=1e-9)


def test_filter_step_pulls_estimate_towards_truth():
    truth = np.array([0.2, 0.1, 0.05])
    x = truth + np.array([0.05, -0.04, 0.03])
    P = np.eye(3) * 0.01
    Z = [measurementFunction(truth, m)[0] for m in BOX_MAP]
    R = [np.diag([0.05 ** 2, 0.02 ** 2])] * len(Z)
    v, H, R_hat = associateMeasurements(x, P, Z, R, BOX_MAP, 3.0)
    x_new, P_new = filterStep(x, P, v, H, R_hat)
    assert np.linalg.norm(x_new - truth) < 0.2 * np.linalg.norm(x - truth)
    assert np.trace(P_new) < np.trace(P)


def test_closed_loop_bounds_drift():
    """Biased odometry (3 % on the left wheel) with and without line updates."""
    rng = np.random.default_rng(1)

    def run(use_updates):
        xt = np.zeros(3)
        xe = np.zeros(3)
        P = np.eye(3) * 1e-3
        for k in range(1500):
            w = 0.03 if (k // 250) % 2 else 0.0
            u_true = np.array([0.004 - w * B, 0.004 + w * B])
            xt, _, _ = transitionFunction(xt, u_true, B)
            u = u_true * [1.03, 1.0] + rng.normal(0, 2e-4, 2)
            xe, Fx, Fu = transitionFunction(xe, u, B)
            P = predictCovariance(P, Fx, Fu, u[0], u[1], 0.01)
            if use_updates and k % 5 == 0:
                lines = split_and_merge_iterative(raycast_box(xt, noise=0.01), 0.08)
                Z = [np.array([theta, rho]) for rho, theta, _, _ in lines]
                R = [np.diag([0.05 ** 2, 0.02 ** 2])] * len(Z)
                v, H, R_hat = associateMeasurements(xe, P, Z, R, BOX_MAP, 3.0)
                xe, P = filterStep(xe, P, v, H, R_hat)
        return np.hypot(*(xe[:2] - xt[:2]))

    assert run(False) > 0.3
    assert run(True) < 0.05
