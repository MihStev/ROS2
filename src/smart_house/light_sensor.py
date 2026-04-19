import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import random

class Light_Sensor(Node):
    def __init__(self):
        super().__init__('light_sensor')
        self.publisher = self.create_publisher(Int64, 'home/lighting', 10)
        self.timer = self.create_timer(1.0, self.on_timer)
    def on_timer(self):
        msg = Int64()
        msg.data = round(random.uniform(0, 1000))
        self.publisher.publish(msg)
        self.get_logger().info(f'Lighting: {msg.data}')

def main():
    rclpy.init()
    light_sensor = Light_Sensor()
    rclpy.spin(light_sensor)

    light_sensor.destroy_node()
    rclpy.shutdown()