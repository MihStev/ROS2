import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random
class Temp_Sensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        self.publisher = self.create_publisher(Float64, 'home/temperature', 10)
        self.timer = self.create_timer(1.0, self.on_timer)
    def on_timer(self):
        msg = Float64()
        msg.data = round(random.uniform(18.0, 30.0),1)
        self.publisher.publish(msg)
        self.get_logger().info(f'Temperature: {msg.data}')
def main():
    rclpy.init()
    temperuture_sensor = Temp_Sensor()
    rclpy.spin(temperuture_sensor)

    temperuture_sensor.destroy_node()
    rclpy.shutdown()