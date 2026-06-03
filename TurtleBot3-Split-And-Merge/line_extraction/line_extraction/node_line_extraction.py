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

    for i in range(begin_ind+1, end_ind):
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

def line_parameters(p1, p2):

    x1, y1 = p1
    x2, y2 = p2

    dx = x2 - x1
    dy = y2 - y1

    nx = -dy
    ny = dx

    norm = math.sqrt(nx**2 + ny**2)

    if norm == 0:
        return 0, 0

    nx /= norm
    ny /= norm

    rho = nx*x1 + ny*y1
    theta = math.atan2(ny, nx)

    return rho, theta



def split_and_merge_iterative(points, threshold):
    segment_queue = []
    segment_queue.append((0, len(points)-1))
    lines = []

    while len(segment_queue) > 0:
        i,j = segment_queue.pop()
        line_fits, k = do_line_fits(points, threshold, i, j)
        if line_fits:
            if i + 4 < j:
                rho, theta = line_parameters(points[i], points[j])
                lines.append((rho,theta, i, j))
        else:
            segment_queue.append((i,k))
            segment_queue.append((k,j))
    
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
            self.get_logger().info(f"Line {i+1}: rho={lines[i][0]:.3f}, theta={lines[i][1]:.3f}")
        
        self.get_logger().info(f"{len(lines)} lines found, time: {exec_time*1000:.3f} ms")

        


def main(args=None):
    rclpy.init(args=args)

    node = SplitAndMergeNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()