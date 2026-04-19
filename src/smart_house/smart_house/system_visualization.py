import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Int64

class System_Visualization(Node):
    def __init__(self):
        super().__init__('system_visuals')
        self.temp = None
        self.hum = None
        self.light = None

        self.subscription1 = self.create_subscription(Float64, 'home/temperature', self.system_callback1, 10)
        self.subscription2 = self.create_subscription(Float64, 'home/humidity', self.system_callback2, 10)
        self.subscription3 = self.create_subscription(Int64, 'home/lighting', self.system_callback3, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def system_callback1(self, msg):
        self.temp = msg.data

    def system_callback2(self, msg):
        self.hum = msg.data

    def system_callback3(self, msg):
        self.light = msg.data

    def timer_callback(self):
        if all(v is not None for v in [self.temp, self.hum, self.light]):
            self.get_logger().info(f'Temperature: {self.temp}°C| Humidity: {self.hum}% | Lighting: {self.light} lux ')


def main():
    rclpy.init()
    system_visuals = System_Visualization()
    rclpy.spin(system_visuals)

    system_visuals.destroy_node()
    rclpy.shutdown()