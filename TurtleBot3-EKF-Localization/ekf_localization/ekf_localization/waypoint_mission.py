import rclpy
from rclpy.node import Node

import math
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('waypoint_mission')

        # False - koristi se /odom
        # True  - koristi se /ekf_pose
        self.ekf_active = self.declare_parameter('use_ekf', True).value

        # referentne poze u frejmu map (kvadrat 1 m x 1 m od spawna u turtlebot3_maze)
        wp = self.declare_parameter(
            'waypoints', [-1.0, -0.5, -1.0, 0.5, -2.0, 0.5, -2.0, -0.5]).value
        self.waypoints = list(zip(wp[0::2], wp[1::2]))

        # poza odom frejma u map frejmu (pocetna poza robota); koristi se samo
        # kada je povratna sprega preko /odom, da bi referentne poze bile u istom frejmu
        self.odom_in_map = (
            self.declare_parameter('odom_x', -2.0).value,
            self.declare_parameter('odom_y', -0.5).value,
            self.declare_parameter('odom_yaw', 0.0).value,
        )

        self.tolerance = self.declare_parameter('tolerance', 0.05).value

        # ogranicenja brzina TurtleBot3 Burger
        self.max_linear = 0.22
        self.max_angular = 2.84

        self.current_waypoint = 0
        self.mission_finished = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.vel_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.ekf_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/ekf_pose',
            self.ekf_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info('Robot Controller Node has been started.')

    def odom_callback(self, msg: Odometry):

        if self.ekf_active:
            return

        q = msg.pose.pose.orientation

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)

        ox, oy, oth = self.odom_in_map
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.current_x = ox + x * math.cos(oth) - y * math.sin(oth)
        self.current_y = oy + x * math.sin(oth) + y * math.cos(oth)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp) + oth

    def ekf_callback(self, msg: PoseWithCovarianceStamped):

        if not self.ekf_active:
            return

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)

        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    def on_timer(self):

        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()

        if self.mission_finished:
            self.vel_pub.publish(vel_msg)
            return

        self.target_x = self.waypoints[self.current_waypoint][0]
        self.target_y = self.waypoints[self.current_waypoint][1]

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y

        vel_const = 0.2
        rho = math.sqrt(dx**2 + dy**2)

        moving_forward = True

        if rho < self.tolerance:

            self.get_logger().info(
                f'Target reached! waypoint {self.current_waypoint} '
                f'({self.target_x:.2f}, {self.target_y:.2f}), '
                f'estimate ({self.current_x:.3f}, {self.current_y:.3f})')

            self.current_waypoint += 1

            if self.current_waypoint >= len(self.waypoints):
                self.mission_finished = True

            self.vel_pub.publish(vel_msg)
            return

        else:

            alpha = -self.current_theta + math.atan2(dy, dx)
            beta = -self.current_theta - alpha

            while alpha > math.pi:
                alpha -= 2 * math.pi

            while alpha < -math.pi:
                alpha += 2 * math.pi

            if abs(alpha) > math.pi / 2:
                moving_forward = False
                alpha = alpha - math.copysign(math.pi, alpha)

            k_rho = 0.5
            k_alpha = 0.8
            k_beta = -0.2

            vel_linear = k_rho * rho
            vel_angular = k_alpha * alpha + k_beta * beta

            if vel_linear > 0.001:
                vel_angular = (vel_angular / vel_linear) * vel_const
            else:
                vel_angular = 0.0

            if moving_forward:
                vel_linear = vel_const
            else:
                vel_linear = -vel_const

            vel_msg.twist.linear.x = max(-self.max_linear, min(self.max_linear, vel_linear))
            vel_msg.twist.angular.z = max(-self.max_angular, min(self.max_angular, vel_angular))

        self.vel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)

    robot_controller = RobotControllerNode()

    rclpy.spin(robot_controller)

    robot_controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()