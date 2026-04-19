import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random
class Humidity_Sensor(Node):
    def __init__(self):
        super().__init__('humidity_sensor')
        self.publisher = self.create_publisher(Float64, 'home/humidity', 10)
        self.timer = self.create_timer(1.0, self.on_timer)
    def on_timer(self):
        msg = Float64()
        msg.data = round(random.uniform(30.0, 70.0), 1)
        self.publisher.publish(msg)
        self.get_logger().info(f'Humidity: {msg.data}')

def main():
    rclpy.init()
    humidity_sensor = Humidity_Sensor()
    rclpy.spin(humidity_sensor)


    humidity_sensor.destroy_node()
    rclpy.shutdown()