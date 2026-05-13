import rclpy
from rclpy.node import Node

import math
from turtlebot3_interfaces.srv import SetMode 
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')

        self.srv = self.create_service(SetMode, 'set_mode', self.set_mode_callback)

        self.current_mode = 0 # 0 = IDLE, 1 = MANUAL, 2 = AUTO
        self.manual_cmd = ""   
        # Current manual command (e.g., "FORWARD", "BACKWARD", "LEFT", "RIGHT")
        # 'w', 's', 'a', 'd' for forward, backward, left, right respectively
        self.target_x = 0.0
        self.target_y = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0


        self.vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info('Robot Controller Node has been started.')

    def set_mode_callback(self, request, response):
        self.current_mode = request.mode
        self.manual_cmd = request.manual_cmd
        self.target_x = request.target_x
        self.target_y = request.target_y

        self.get_logger().info(f'Setting mode to: {self.current_mode}')
        response.success = True
        response.message = f"Mode set to {self.current_mode} with manual_cmd: {self.manual_cmd}, \
        target_x: {self.target_x}, target_y: {self.target_y}"
        return response

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)
        self.get_logger().info(f'Current position: x={self.current_x}, y={self.current_y}, theta={self.current_theta}')

    def on_timer(self):
        vel_msg = TwistStamped()

        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = "base_link"

        if self.current_mode == 1:  # MANUAL mode
            if self.manual_cmd == 'w':  # FORWARD
                vel_msg.twist.linear.x = 0.2
            elif self.manual_cmd == 's':  # BACKWARD
                vel_msg.twist.linear.x = -0.2
            elif self.manual_cmd == 'a':  # LEFT
                vel_msg.twist.angular.z = 0.2
            elif self.manual_cmd == 'd':  # RIGHT
                vel_msg.twist.angular.z = -0.2
            elif self.manual_cmd == 'x':
                vel_msg.twist.linear.x = 0.0
                vel_msg.twist.angular.z = 0.0
        elif self.current_mode == 2:  # AUTO mode
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            vel_const = 0.2
            rho = math.sqrt(dx**2 + dy**2)

            # if rho < 0.05:
            #     self.get_logger().info('Target reached!')
            #     self.current_mode = 0
            # else:
            #     alpha = math.atan2(dy, dx) - self.current_theta
            #     alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # normalize to [-pi, pi]

            #     moving_forward = True
            #     # If the target is behind the robot, it's more efficient to turn around and drive backwards
            #     if abs(alpha) > math.pi / 2:
            #         moving_forward = False
            #         alpha = alpha - math.copysign(math.pi, alpha)

            #     if abs(alpha) > 0.1:  # Phase 1: rotate to face target (or back to target)
            #         vel_msg.twist.linear.x = 0.0
            #         vel_msg.twist.angular.z = 1.0 * alpha
            #     else:                  # Phase 2: drive straight with small heading correction
            #         direction = 1.0 if moving_forward else -1.0
            #         vel_msg.twist.linear.x = direction * min(0.5 * rho, vel_const)
            #         vel_msg.twist.angular.z = 0.5 * alpha
            
            # Differential drive control using the unicycle model
            # ---------------------------------------------------------------
            moving_forward = True
            if rho < 0.05:
                self.get_logger().info('Target reached!')
                self.current_mode = 0 
            else:
                alpha = -self.current_theta + math.atan2(dy, dx)
                beta = -self.current_theta - alpha

                # Normalize alpha to be within [-pi, pi]
                while alpha > math.pi:
                    alpha -= 2 * math.pi
                while alpha < -math.pi:
                    alpha += 2 * math.pi

                # Check if target is behind the robot
                if abs(alpha) > math.pi / 2:
                    moving_forward = False
                    alpha = alpha - math.copysign(math.pi, alpha)

                # Choosing gains for the controller
                k_rho = 0.5  
                k_alpha = 1.5 
                k_beta = -0.5

                # Calculate BASE (theoretical) velocities for scaling
                vel_linear = k_rho * rho
                vel_angular = k_alpha * alpha + k_beta * beta

                # Scale angular velocity to maintain the path with constant linear speed
                if vel_linear > 0.001:
                    vel_angular = (vel_angular/vel_linear) * vel_const
                else:
                    vel_angular = 0.0


                # Set final constant linear velocity based on direction
                if moving_forward:
                    vel_linear = vel_const
                else:
                    vel_linear = -vel_const
                vel_msg.twist.linear.x = vel_linear
                vel_msg.twist.angular.z = vel_angular
                # --------------------------------------------------------------
        # Send velocity command
        self.vel_pub.publish(vel_msg)
        
def main(args = None):
    rclpy.init(args = args)
    robot_controller = RobotControllerNode()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
