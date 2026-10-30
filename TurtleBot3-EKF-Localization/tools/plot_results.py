"""Plots and numbers for runs recorded with record_trajectories.py (all in the map frame).

One run: trajectories + estimation error (+ optional covariance / cmd_vel figure):

    python3 tools/plot_results.py run_b.csv --map ekf_localization/maps/maze_lines.yaml \
        --out docs/run_b.png --details docs/run_b_details.png --title "(b) /ekf_pose"

Several runs: ground-truth paths and waypoint accuracy side by side:

    python3 tools/plot_results.py a.csv b.csv c.csv --labels "(a)" "(b)" "(c)" \
        --map ekf_localization/maps/maze_lines.yaml --out docs/compare.png
"""
import argparse
import csv
import math
from collections import defaultdict

import matplotlib
import numpy as np
import yaml

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

COLORS = {'ground_truth': '#222222', 'odom': '#e07b00', 'ekf': '#1f77b4'}
LABELS = {'ground_truth': 'Ground truth (Gazebo)', 'odom': 'Wheel odometry (/odom)',
          'ekf': 'EKF (/ekf_pose)'}
RUN_COLORS = ['#e07b00', '#1f77b4', '#9467bd', '#2ca02c', '#d62728']


def load(path):
    data = defaultdict(list)
    with open(path) as f:
        for r in csv.DictReader(f):
            row = [float(r[k]) if r[k] != '' else math.nan
                   for k in ('t', 'x', 'y', 'theta', 'var_x', 'var_y', 'var_theta')]
            data[r['source']].append(row)
    return {k: np.array(v) for k, v in data.items()}


def gt_at(gt, t):
    return np.array([np.interp(t, gt[:, 0], gt[:, i]) for i in (1, 2)])


def errors(est, gt):
    """Position / heading error of `est` against ground truth interpolated at est times."""
    t = est[:, 0]
    mask = (t >= gt[0, 0]) & (t <= gt[-1, 0])
    t, est = t[mask], est[mask]
    gx = np.interp(t, gt[:, 0], gt[:, 1])
    gy = np.interp(t, gt[:, 0], gt[:, 2])
    gth = np.interp(t, gt[:, 0], np.unwrap(gt[:, 3]))
    pos = np.hypot(est[:, 1] - gx, est[:, 2] - gy)
    head = np.abs(np.remainder(est[:, 3] - gth + math.pi, 2 * math.pi) - math.pi)
    return t, pos, head


def reach_times(d):
    """Sim times at which the controller declared a waypoint reached.

    The mission publishes one zero command when a waypoint is reached (and zeros
    for good once the last one is), so these are the moving -> zero transitions
    of /cmd_vel. The /rosout stamps of the log line are wall-clock time.
    """
    c = d.get('cmd_vel')
    if c is None:
        return []
    zero = (c[:, 1] == 0) & (c[:, 2] == 0)
    return list(c[1:][zero[1:] & ~zero[:-1], 0])


def waypoint_errors(d):
    """Ground-truth distance to the reference when the controller declared it reached."""
    if 'waypoint' not in d:
        return []
    out = []
    for t, w in zip(reach_times(d), d['waypoint']):
        out.append((int(w[3]), w[1], w[2], np.hypot(*(gt_at(d['ground_truth'], t) - w[1:3])), t))
    return out


def draw_map(ax, path):
    with open(path) as f:
        walls = yaml.safe_load(f)['walls']
    for a, r in walls.values():
        # infinite map line x cos a + y sin a = r, drawn across the plot area
        c, s = math.cos(a), math.sin(a)
        p0 = np.array([r * c, r * s])
        dvec = np.array([-s, c])
        pts = np.array([p0 + k * dvec for k in (-8, 8)])
        ax.plot(pts[:, 0], pts[:, 1], color='#bbbbbb', lw=0.8, ls='--', zorder=0)


def frame_axes(ax, xy, waypoints):
    ax.plot(*np.vstack([waypoints, waypoints[:1]]).T, 'o', color='#2ca02c', ms=8, mfc='none',
            mew=1.5, label='Waypoints')
    lo, hi = xy.min(0) - 0.4, xy.max(0) + 0.4
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect('equal')
    ax.set_xlabel('x [m] (map frame)')
    ax.set_ylabel('y [m]')
    ax.grid(alpha=0.3)


def summary(name, pos, head):
    rmse = math.sqrt(np.mean(pos ** 2))
    return (f'{name:5s}  RMSE pos = {rmse:.3f} m   max pos = {pos.max():.3f} m   '
            f'final pos = {pos[-1]:.3f} m   final heading = {math.degrees(head[-1]):.1f} deg')


def single(args, d, waypoints):
    gt = d['ground_truth']
    t0 = gt[0, 0]
    fig = plt.figure(figsize=(12, 5.2))
    ax = fig.add_subplot(1, 2, 1)
    if args.map:
        draw_map(ax, args.map)
    for k in ('ground_truth', 'odom', 'ekf'):
        if k in d:
            ax.plot(d[k][:, 1], d[k][:, 2], color=COLORS[k], label=LABELS[k],
                    lw=2.2 if k == 'ground_truth' else 1.4)
    for i, (*_, t) in enumerate(waypoint_errors(d)):
        ax.plot(*gt_at(gt, t), 'x', color='#d62728', ms=8, mew=2,
                label='Ground truth when "reached"' if i == 0 else None)
    frame_axes(ax, np.vstack([d[k][:, 1:3] for k in ('ground_truth', 'odom', 'ekf') if k in d]),
               waypoints)
    ax.legend(fontsize=8, loc='best')
    ax.set_title('Trajectories (dashed: map lines)', fontsize=9)

    ax2 = fig.add_subplot(2, 2, 2)
    ax3 = fig.add_subplot(2, 2, 4, sharex=ax2)
    for k in ('odom', 'ekf'):
        if k not in d:
            continue
        t, pos, head = errors(d[k], gt)
        ax2.plot(t - t0, pos, color=COLORS[k], label=LABELS[k])
        ax3.plot(t - t0, np.degrees(head), color=COLORS[k])
        print(summary(k, pos, head))
    ax2.set_ylabel('position error [m]')
    ax3.set_ylabel('heading error [deg]')
    ax3.set_xlabel('simulation time [s]')
    for a in (ax2, ax3):
        a.grid(alpha=0.3)
    ax2.legend(fontsize=8)
    fig.suptitle(args.title)
    fig.tight_layout()
    fig.savefig(args.out, dpi=130)

    for i, wx, wy, e, _ in waypoint_errors(d):
        print(f'waypoint {i} ({wx:+.2f}, {wy:+.2f}): ground truth {e * 100:.1f} cm away when reached')

    if args.details and 'ekf' in d:
        details(args, d, t0)


def details(args, d, t0):
    ekf, odom = d['ekf'], d.get('odom')
    if odom is not None and not np.any(odom[:, 4:7]):
        odom = None  # Gazebo publishes an all-zero covariance on /odom
    # the covariance only shrinks in a correction step
    tr = ekf[:, 4] + ekf[:, 5]
    corr = np.where(np.diff(tr) < 0)[0] + 1
    print(f'corrections: {len(corr)}')
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axs[0].plot(ekf[:, 0] - t0, np.sqrt(ekf[:, 4]) * 100, label=r'EKF $\sigma_x$')
    axs[0].plot(ekf[:, 0] - t0, np.sqrt(ekf[:, 5]) * 100, label=r'EKF $\sigma_y$')
    if odom is not None:
        axs[0].plot(odom[:, 0] - t0, np.sqrt(odom[:, 4]) * 100, '--', color='#e07b00',
                    label=r'/odom $\sigma_x$ (Gazebo)')
    axs[0].set_ylabel('std. dev. [cm]')
    axs[1].plot(ekf[:, 0] - t0, np.degrees(np.sqrt(ekf[:, 6])), label=r'EKF $\sigma_\theta$')
    if odom is not None:
        axs[1].plot(odom[:, 0] - t0, np.degrees(np.sqrt(odom[:, 6])), '--', color='#e07b00',
                    label=r'/odom $\sigma_\theta$ (Gazebo)')
    axs[1].set_ylabel('std. dev. [deg]')
    if odom is None:
        axs[0].text(0.01, 0.04, '/odom covariance from Gazebo: all zeros (not shown)',
                    transform=axs[0].transAxes, fontsize=8, color='#e07b00')
    if 'cmd_vel' in d:
        c = d['cmd_vel']
        axs[2].step(c[:, 0] - t0, c[:, 2], where='post', color='#444444', lw=1,
                    label=r'$\omega$ command [rad/s]')
        axs[2].step(c[:, 0] - t0, c[:, 1] * 10, where='post', color='#2ca02c', lw=1,
                    label='v command x10 [m/s]')
    for k, ti in enumerate(ekf[corr, 0] - t0):
        axs[2].axvline(ti, color='#1f77b4', alpha=0.12, lw=0.8,
                       label='EKF correction' if k == 0 else None)
    axs[2].set_xlabel('simulation time [s]')
    for a in axs:
        a.grid(alpha=0.3)
        a.legend(fontsize=8, loc='upper right')
    axs[0].set_yscale('log')
    axs[1].set_yscale('log')
    fig.suptitle(args.title + ' - covariance and control signal')
    fig.tight_layout()
    fig.savefig(args.details, dpi=130)


def compare(args, runs, waypoints):
    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    if args.map:
        draw_map(ax, args.map)
    xy = []
    for k, (label, d) in enumerate(zip(args.labels, runs)):
        gt = d['ground_truth']
        ax.plot(gt[:, 1], gt[:, 2], color=RUN_COLORS[k], lw=1.8, label=label)
        xy.append(gt[:, 1:3])
        errs = waypoint_errors(d)
        for *_, t in errs:
            ax.plot(*gt_at(gt, t), 'x', color=RUN_COLORS[k], ms=8, mew=2)
        es = ', '.join(f'{e[3] * 100:.1f}' for e in errs)
        mean = f'  (mean {np.mean([e[3] for e in errs]) * 100:.1f})' if errs else ''
        print(f'{label:40s} waypoint error when reached [cm]: {es}{mean}')
    frame_axes(ax, np.vstack(xy), waypoints)
    ax.legend(fontsize=8, loc='best')
    ax.set_title(args.title or 'Ground-truth paths (x: ground truth when the controller '
                 'declared the waypoint reached)', fontsize=9)
    fig.tight_layout()
    fig.savefig(args.out, dpi=130)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='+')
    ap.add_argument('--labels', nargs='+')
    ap.add_argument('--map')
    ap.add_argument('--out', required=True)
    ap.add_argument('--details')
    ap.add_argument('--title', default='')
    ap.add_argument('--waypoints', type=float, nargs='+',
                    default=[-1.0, -0.5, -1.0, 0.5, -2.0, 0.5, -2.0, -0.5])
    args = ap.parse_args()
    waypoints = np.array(args.waypoints).reshape(-1, 2)
    runs = [load(p) for p in args.csv]
    if len(runs) == 1:
        single(args, runs[0], waypoints)
    else:
        args.labels = args.labels or args.csv
        compare(args, runs, waypoints)


if __name__ == '__main__':
    main()
