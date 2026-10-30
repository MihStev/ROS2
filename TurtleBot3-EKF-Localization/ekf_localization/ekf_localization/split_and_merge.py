import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


def do_line_fits(points, threshold, begin_ind, end_ind):

    x1, y1 = points[begin_ind]
    x2, y2 = points[end_ind]

    max_dist = 0
    max_ind = -1

    for i in range(begin_ind + 1, end_ind):
        x0, y0 = points[i]
        num = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
        dem = math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)

        if dem == 0:
            return True, 0

        dist = num / dem

        if dist > max_dist:
            max_dist = dist
            max_ind = i

    if max_dist > threshold:
        return False, max_ind

    return True, 0


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def line_parameters(p1, p2):

    x1, y1 = p1
    x2, y2 = p2

    dx = x2 - x1
    dy = y2 - y1

    nx = -dy
    ny = dx

    norm = math.sqrt(nx ** 2 + ny ** 2)

    if norm == 0:
        return 0.0, 0.0

    nx /= norm
    ny /= norm

    rho = nx * x1 + ny * y1
    theta = math.atan2(ny, nx)

    if rho < 0:
        rho = -rho
        theta = theta + math.pi

    theta = normalize_angle(theta)

    return rho, theta


def angle_diff(a, b):
    return abs(normalize_angle(a - b))


def merge_similar_lines(points, lines, angle_threshold=0.12, rho_threshold=0.20):
    """
    Spaja susedne linije koje su vrlo slicne.

    lines format:
        (rho, theta, i, j)
    """

    if len(lines) == 0:
        return []

    lines = sorted(lines, key=lambda line: line[2])

    merged = []
    current = lines[0]

    for next_line in lines[1:]:

        rho1, theta1, i1, j1 = current
        rho2, theta2, i2, j2 = next_line

        similar_angle = angle_diff(theta1, theta2) < angle_threshold
        similar_rho = abs(rho1 - rho2) < rho_threshold
        close_indices = abs(i2 - j1) <= 5

        if similar_angle and similar_rho and close_indices:
            new_i = i1
            new_j = j2

            new_rho, new_theta = line_parameters(
                points[new_i],
                points[new_j]
            )

            current = (new_rho, new_theta, new_i, new_j)

        else:
            merged.append(current)
            current = next_line

    merged.append(current)

    # kruzni merge: poslednja i prva linija u lidar skenu
    if len(merged) > 1:
        first = merged[0]
        last = merged[-1]

        rho1, theta1, i1, j1 = last
        rho2, theta2, i2, j2 = first

        similar_angle = angle_diff(theta1, theta2) < angle_threshold
        similar_rho = abs(rho1 - rho2) < rho_threshold

        if similar_angle and similar_rho:
            new_i = i1
            new_j = j2

            new_rho, new_theta = line_parameters(
                points[new_i],
                points[new_j]
            )

            merged[0] = (new_rho, new_theta, new_i, new_j)
            merged.pop()

    return merged


def split_and_merge_iterative(points, threshold):
    segment_queue = []
    segment_queue.append((0, len(points) - 1))
    lines = []

    while len(segment_queue) > 0:
        i, j = segment_queue.pop()
        line_fits, k = do_line_fits(points, threshold, i, j)

        if line_fits:
            if i + 3 < j:
                rho, theta = line_parameters(points[i], points[j])
                lines.append((rho, theta, i, j))
        else:
            segment_queue.append((i, k))
            segment_queue.append((k, j))

    lines = merge_similar_lines(points, lines)

    return lines


class SplitAndMergeNode(Node):
    def __init__(self):
        super().__init__('split_and_merge_node')

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.publisher = self.create_publisher(
            MarkerArray, '/lines', 10
        )

        self.threshold = 0.08

    def polar_to_decart(self, msg):

        points = []

        for i in range(len(msg.ranges)):

            ro = msg.ranges[i]

            if not math.isfinite(ro):
                continue

            if ro < msg.range_min or ro > msg.range_max:
                continue

            theta = msg.angle_min + i * msg.angle_increment

            x = ro * math.cos(theta)
            y = ro * math.sin(theta)

            points.append((x, y))

        return points

    def scan_callback(self, msg):
        start_time = time.time()

        points = self.polar_to_decart(msg)

        lines = split_and_merge_iterative(points, self.threshold)

        marker_array = MarkerArray()

        for i in range(len(lines)):

            line = lines[i]

            j = line[2]
            k = line[3]

            marker = Marker()
            marker.header.frame_id = msg.header.frame_id
            marker.header.stamp = msg.header.stamp

            marker.id = i
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD

            marker.scale.x = 0.03

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            p1 = Point()
            p1.x = points[j][0]
            p1.y = points[j][1]
            p1.z = 0.0

            p2 = Point()
            p2.x = points[k][0]
            p2.y = points[k][1]
            p2.z = 0.0

            marker.points.append(p1)
            marker.points.append(p2)

            marker_array.markers.append(marker)

        self.publisher.publish(marker_array)
        end_time = time.time()
        exec_time = end_time - start_time

        for i in range(len(lines)):
            self.get_logger().info(
                f"Line {i + 1}: rho={lines[i][0]:.3f}, "
                f"theta={lines[i][1]:.3f}, "
                f"{lines[i][2]} ----- {lines[i][3]}"
            )

        self.get_logger().info(
            f"{len(lines)} lines found, time: {exec_time * 1000:.3f} ms"
        )


def main(args=None):
    rclpy.init(args=args)

    node = SplitAndMergeNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()