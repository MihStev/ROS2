"""Replay a recorded bag through the EKF functions offline and compare with ground truth.

The bag needs /joint_states, /scan and /ground_truth/poses (see README):

    python3 tools/replay_bag.py bag/ --map ekf_localization/maps/maze_lines.yaml --gate 3 2 1

Everything is in the map (= Gazebo world) frame; the filter starts at --init (the spawn pose).

For every accepted association it also checks the true pose: an observed line
counts as "no map match" if no map line is within a squared Mahalanobis distance
of 25 (about 5 sigma) when evaluated at the ground-truth pose.
"""
import argparse
import math

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState, LaserScan
from tf2_msgs.msg import TFMessage

from ekf_localization.data_association import associateMeasurements
from ekf_localization.ekf_update import filterStep
from ekf_localization.measurement_model import measurementFunction
from ekf_localization.motion_model import predictCovariance, transitionFunction
from ekf_localization.split_and_merge import split_and_merge_iterative

TYPES = {'/scan': LaserScan, '/joint_states': JointState, '/ground_truth/poses': TFMessage}


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    events, gt = [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic not in TYPES:
            continue
        msg = deserialize_message(data, TYPES[topic])
        t *= 1e-9
        if topic == '/ground_truth/poses':
            tf = msg.transforms[0]
            q = tf.transform.rotation
            gt.append((t, tf.transform.translation.x, tf.transform.translation.y,
                       math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))))
        else:
            events.append((t, topic, msg))
    # a bag recovered with `ros2 bag reindex` has no message index and is not read in order
    events.sort(key=lambda e: e[0])
    gt.sort()
    # keep only the first simulation if the simulator was restarted during the recording
    last = None
    for k, (t, topic, msg) in enumerate(events):
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if topic == '/joint_states':
            if last is not None and st < last - 1.0:
                events = events[:k]
                gt = [g for g in gt if g[0] < t]
                break
            last = st
    return events, np.array(gt)


def scan_lines(msg, threshold=0.08, lidar_offset_x=-0.032):
    pts = []
    for i, ro in enumerate(msg.ranges):
        if math.isfinite(ro) and msg.range_min <= ro <= msg.range_max:
            th = msg.angle_min + i * msg.angle_increment
            pts.append((ro * math.cos(th) + lidar_offset_x, ro * math.sin(th)))
    if len(pts) < 2:
        return []
    return [np.array([theta, rho]) for rho, theta, _, _ in split_and_merge_iterative(pts, threshold)]


def nearest(x, P, z, R, M):
    best = (None, math.inf)
    for i, m in enumerate(M):
        zh, H = measurementFunction(x, m)
        v = z - zh
        v[0] = math.remainder(v[0], 2 * math.pi)
        d = v @ np.linalg.inv(H @ P @ H.T + R) @ v
        if d < best[1]:
            best = (i, d)
    return best


def replay(events, gt, M, gate, x0, sigma_alpha=0.05, sigma_r=0.02, use_scan=True, b=0.08):
    def gt_at(t):
        return np.array([np.interp(t, gt[:, 0], gt[:, i]) for i in (1, 2, 3)])

    x, P, prev = np.array(x0, float), np.eye(3) * 1e-3, None
    errs, n_assoc, ok, wrong, unmapped = [], [], 0, 0, 0
    R = np.diag([sigma_alpha ** 2, sigma_r ** 2])
    for t, topic, msg in events:
        if topic == '/joint_states':
            phi = (msg.position[msg.name.index('wheel_left_joint')],
                   msg.position[msg.name.index('wheel_right_joint')])
            if prev is not None:
                u = 0.033 * np.array([phi[0] - prev[0], phi[1] - prev[1]])
                x, Fx, Fu = transitionFunction(x, u, b)
                P = predictCovariance(P, Fx, Fu, u[0], u[1], 0.01)
            prev = phi
            continue
        if not use_scan:
            continue
        Z = scan_lines(msg)
        if not Z:
            continue
        x_true = gt_at(t)
        for z in Z:
            i, d = nearest(x, P, z, R, M)
            if d < gate ** 2:
                i_true, d_true = nearest(x_true, np.eye(3) * 1e-9, z, R, M)
                if d_true >= 25:
                    unmapped += 1
                elif i_true == i:
                    ok += 1
                else:
                    wrong += 1
        v, H, R_hat = associateMeasurements(x, P, Z, [R] * len(Z), M, gate)
        n_assoc.append(0 if v is None else len(v) // 2)
        x, P = filterStep(x, P, v, H, R_hat)
        errs.append(np.hypot(*(x[:2] - x_true[:2])))
    final = np.hypot(*(x[:2] - gt_at(events[-1][0])[:2]))
    return final, np.array(errs), np.array(n_assoc), ok, wrong, unmapped


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--map', required=True)
    ap.add_argument('--gate', type=float, nargs='+', default=[3.0])
    ap.add_argument('--init', type=float, nargs=3, default=[-2.0, -0.5, 0.0],
                    metavar=('X', 'Y', 'YAW'), help='initial pose in the map frame')
    ap.add_argument('--half-wheel-separation', type=float, default=0.08,
                    help='b used by the filter (0.08 is the true value)')
    args = ap.parse_args()

    events, gt = read_bag(args.bag)
    with open(args.map) as f:
        walls = yaml.safe_load(f)['walls']
    M = [np.array(walls[k], float) for k in sorted(walls, key=int)]

    kw = dict(x0=args.init, b=args.half_wheel_separation)
    final, _, _, _, _, _ = replay(events, gt, M, 0, use_scan=False, **kw)
    print(f'dead reckoning only : final position error {final:.3f} m')
    for g in args.gate:
        final, e, n, ok, wrong, unmapped = replay(events, gt, M, g, **kw)
        print(f'gate g = {g:<4}: final {final:.3f} m, RMSE {math.sqrt((e ** 2).mean()):.3f} m, '
              f'max {e.max():.3f} m | lines per scan {n.mean():.2f} (0 in {(n == 0).mean():.0%}) | '
              f'accepted: {ok} correct, {wrong} wrong wall, {unmapped} no map match')


if __name__ == '__main__':
    main()
