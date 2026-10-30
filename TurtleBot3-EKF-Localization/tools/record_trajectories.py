"""Record ground truth, wheel odometry, the EKF estimate, /cmd_vel and waypoint events to CSV.

Everything is written in the map frame (= Gazebo world frame, the frame of the line map).
/odom starts at the spawn pose, so it is shifted by --odom-origin (the spawn pose).

    python3 tools/record_trajectories.py --out run.csv --odom-origin -2.0 -0.5 0.0

Rows: t, source, x, y, theta, var_x, var_y, var_theta
  source = ground_truth | odom | ekf | cmd_vel (x = v, y = omega) | waypoint (x, y = reference)
"""
import argparse
import csv
import math
import re

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Recorder(Node):

    def __init__(self, out, odom_origin):
        super().__init__('trajectory_recorder',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.odom_origin = odom_origin
        self.file = open(out, 'w', newline='')
        self.csv = csv.writer(self.file)
        self.csv.writerow(['t', 'source', 'x', 'y', 'theta', 'var_x', 'var_y', 'var_theta'])
        self.create_subscription(TFMessage, '/ground_truth/poses', self.on_gt, 50)
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(PoseWithCovarianceStamped, '/ekf_pose', self.on_ekf, 50)
        self.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd, 50)
        self.create_subscription(Log, '/rosout', self.on_log, 50)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gt(self, msg):
        # The bridge drops entity names; the first entry of dynamic_pose/info is
        # the model itself (in the world frame), the rest are its links.
        # Its header stamp is zero, so the node clock (sim time) is used.
        for tf in msg.transforms[:1]:
            p = tf.transform.translation
            self.csv.writerow([self.now(), 'ground_truth', p.x, p.y,
                               yaw(tf.transform.rotation), '', '', ''])

    def on_odom(self, msg):
        p = msg.pose.pose
        c = msg.pose.covariance
        x0, y0, th0 = self.odom_origin
        cs, sn = math.cos(th0), math.sin(th0)
        x, y = p.position.x, p.position.y
        self.csv.writerow([stamp(msg.header), 'odom', x0 + cs * x - sn * y, y0 + sn * x + cs * y,
                           math.remainder(yaw(p.orientation) + th0, 2 * math.pi),
                           c[0], c[7], c[35]])

    def on_ekf(self, msg):
        p = msg.pose.pose
        c = msg.pose.covariance
        self.csv.writerow([stamp(msg.header), 'ekf', p.position.x, p.position.y,
                           yaw(p.orientation), c[0], c[7], c[35]])

    def on_cmd(self, msg):
        self.csv.writerow([self.now(), 'cmd_vel', msg.twist.linear.x, msg.twist.angular.z,
                           '', '', '', ''])

    def on_log(self, msg):
        m = re.search(r'Target reached! waypoint (\d+) \(([-\d.]+), ([-\d.]+)\)', msg.msg)
        if m:
            # rosout stamps are wall-clock time; the reach time is taken from /cmd_vel
            self.csv.writerow([self.now(), 'waypoint', float(m.group(2)), float(m.group(3)),
                               int(m.group(1)), '', '', ''])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--odom-origin', type=float, nargs=3, default=[-2.0, -0.5, 0.0],
                    metavar=('X', 'Y', 'YAW'))
    args = ap.parse_args()
    rclpy.init()
    node = Recorder(args.out, args.odom_origin)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.file.close()


if __name__ == '__main__':
    main()
