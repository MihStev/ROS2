import math
import time
import os
import yaml
from collections import deque
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler

from ament_index_python.packages import get_package_share_directory

from ekf_localization.split_and_merge import split_and_merge_iterative

from ekf_localization.motion_model import (
    transitionFunction,
    predictCovariance,
)

from ekf_localization.data_association import (
    associateMeasurements,
)

from ekf_localization.ekf_update import (
    filterStep,
)


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def load_map_yaml(path):
    with open(path, "r") as file:
        data = yaml.safe_load(file)

    M = []

    for wall_id in sorted(data["walls"].keys(), key=lambda x: int(x)):
        alpha = float(data["walls"][wall_id][0])
        r = float(data["walls"][wall_id][1])
        M.append(np.array([alpha, r]))

    return M


class EKFLineLocalization(Node):

    def __init__(self):
        super().__init__("ekf_line_localization")

        default_map = os.path.join(
            get_package_share_directory("ekf_localization"),
            "maps",
            "maze_lines.yaml"
        )

        # wheel_radius i b (pola razmaka tockova) su za TurtleBot3 Burger
        self.wheel_radius = self.declare_parameter("wheel_radius", 0.033).value
        self.b = self.declare_parameter("half_wheel_separation", 0.08).value
        self.k = self.declare_parameter("odom_noise_k", 0.01).value

        # pocetna poza robota u frejmu mape (za turtlebot3_maze spawn je (-2.0, -0.5))
        self.x = np.array([
            self.declare_parameter("initial_x", 0.0).value,
            self.declare_parameter("initial_y", 0.0).value,
            self.declare_parameter("initial_theta", 0.0).value,
        ])
        self.P = np.eye(3) * 0.001

        self.prev_left_phi = None
        self.prev_right_phi = None

        # istorija koraka predikcije (t, x pre koraka, P pre koraka, u) da bi se
        # popravka primenila na apriornu estimaciju u trenutku skena
        self.history = deque()
        self.history_length = self.declare_parameter("history_length", 2.0).value

        self.threshold = self.declare_parameter("split_threshold", 0.08).value

        # polozaj lidara (base_scan) u odnosu na base_footprint, TurtleBot3 Burger
        self.lidar_offset_x = self.declare_parameter("lidar_offset_x", -0.032).value

        self.sigma_alpha = self.declare_parameter("sigma_alpha", 0.05).value
        self.sigma_r = self.declare_parameter("sigma_r", 0.02).value
        self.g = self.declare_parameter("gate", 3.0).value

        self.scan_enable = self.declare_parameter("use_scan", True).value

        self.map_path = os.path.expanduser(
            self.declare_parameter("map_path", default_map).value
        )

        self.M = load_map_yaml(self.map_path)

        self.get_logger().info(
            f"Loaded {len(self.M)} map lines from {self.map_path}"
        )

        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/ekf_pose",
            10
        )

    def polar_to_decart(self, msg):
        points = []

        for i in range(len(msg.ranges)):
            ro = msg.ranges[i]

            if not math.isfinite(ro):
                continue

            if ro < msg.range_min or ro > msg.range_max:
                continue

            theta = msg.angle_min + i * msg.angle_increment

            x = ro * math.cos(theta) + self.lidar_offset_x
            y = ro * math.sin(theta)

            points.append((x, y))

        return points

    def extract_lines_from_scan(self, msg):
        points = self.polar_to_decart(msg)

        if len(points) < 2:
            return []

        lines = split_and_merge_iterative(
            points,
            self.threshold
        )

        Z = []

        for line in lines:
            rho = line[0]
            theta = line[1]

            Z.append(np.array([theta, rho]))

        return Z

    def joint_callback(self, msg):
        try:
            left_index = msg.name.index("wheel_left_joint")
            right_index = msg.name.index("wheel_right_joint")
        except ValueError:
            return

        left_phi = msg.position[left_index]
        right_phi = msg.position[right_index]

        if self.prev_left_phi is None:
            self.prev_left_phi = left_phi
            self.prev_right_phi = right_phi
            return

        delta_left_phi = left_phi - self.prev_left_phi
        delta_right_phi = right_phi - self.prev_right_phi

        self.prev_left_phi = left_phi
        self.prev_right_phi = right_phi

        delta_sl = self.wheel_radius * delta_left_phi
        delta_sr = self.wheel_radius * delta_right_phi

        u = np.array([delta_sl, delta_sr])

        t = stamp_to_sec(msg.header.stamp)

        self.history.append((t, self.x.copy(), self.P.copy(), u))

        while self.history and self.history[0][0] < t - self.history_length:
            self.history.popleft()

        self.x, self.P = self.predict(self.x, self.P, u)

        self.publish_pose(msg.header.stamp)

    def predict(self, x, P, u):
        x_hat, Fx, Fu = transitionFunction(x, u, self.b)

        P_hat = predictCovariance(
            P,
            Fx,
            Fu,
            u[0],
            u[1],
            self.k
        )

        return x_hat, P_hat

    def scan_callback(self, msg):

        if not self.scan_enable:
            return

        start_time = time.time()

        Z = self.extract_lines_from_scan(msg)

        if len(Z) == 0:
            return

        R_list = []

        for _ in Z:
            R = np.array([
                [self.sigma_alpha ** 2, 0.0],
                [0.0, self.sigma_r ** 2]
            ])
            R_list.append(R)

        # apriorna estimacija u trenutku skena: stanje pre prvog koraka
        # predikcije koji je noviji od vremenske oznake skena
        t_scan = stamp_to_sec(msg.header.stamp)

        later = [h for h in self.history if h[0] > t_scan]

        if later:
            x_scan, P_scan = later[0][1], later[0][2]
        else:
            x_scan, P_scan = self.x, self.P

        v_hat, H_hat, R_hat = associateMeasurements(
            x_scan,
            P_scan,
            Z,
            R_list,
            self.M,
            self.g
        )

        x_post, P_post = filterStep(
            x_scan,
            P_scan,
            v_hat,
            H_hat,
            R_hat
        )

        dx = x_post - x_scan

        # ponovo primeni korake predikcije koji su stigli posle skena
        self.history = deque()

        for t, _, _, u in later:
            self.history.append((t, x_post.copy(), P_post.copy(), u))
            x_post, P_post = self.predict(x_post, P_post, u)

        self.x, self.P = x_post, P_post

        self.publish_pose(self.get_clock().now().to_msg())

        exec_time = (time.time() - start_time) * 1000.0

        if v_hat is None:
            self.get_logger().info(
                f"SCAN: {len(Z)} lines, 0 associated, time={exec_time:.3f} ms"
            )
        else:
            associated_count = int(len(v_hat) / 2)

            self.get_logger().info(
                f"SCAN: {len(Z)} lines, {associated_count} associated, "
                f"correction=[{dx[0]:.3f}, {dx[1]:.3f}, {dx[2]:.3f}], "
                f"time={exec_time:.3f} ms"
            )

    def publish_pose(self, stamp):
        msg = PoseWithCovarianceStamped()

        msg.header.stamp = stamp
        msg.header.frame_id = "map"

        msg.pose.pose.position.x = float(self.x[0])
        msg.pose.pose.position.y = float(self.x[1])
        msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(
            0.0,
            0.0,
            float(self.x[2])
        )

        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        cov = [0.0] * 36

        cov[0] = float(self.P[0, 0])
        cov[1] = float(self.P[0, 1])
        cov[5] = float(self.P[0, 2])

        cov[6] = float(self.P[1, 0])
        cov[7] = float(self.P[1, 1])
        cov[11] = float(self.P[1, 2])

        cov[30] = float(self.P[2, 0])
        cov[31] = float(self.P[2, 1])
        cov[35] = float(self.P[2, 2])

        msg.pose.covariance = cov

        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = EKFLineLocalization()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()