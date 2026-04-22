import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int64


class SystemVisualization(Node):
    def __init__(self):
        super().__init__('system_visuals')

        self.temp = None
        self.hum = None
        self.light = None

        self.new_temp = False
        self.new_hum = False
        self.new_light = False

        self.subscription1 = self.create_subscription(
            Float64, 'home/temperature', self.system_callback1, 10
        )
        self.subscription2 = self.create_subscription(
            Float64, 'home/humidity', self.system_callback2, 10
        )
        self.subscription3 = self.create_subscription(
            Int64, 'home/lighting', self.system_callback3, 10
        )

    def system_callback1(self, msg):
        self.temp = msg.data
        self.new_temp = True
        self.try_display()

    def system_callback2(self, msg):
        self.hum = msg.data
        self.new_hum = True
        self.try_display()

    def system_callback3(self, msg):
        self.light = msg.data
        self.new_light = True
        self.try_display()

    def try_display(self):
        if self.new_temp and self.new_hum and self.new_light:
            self.get_logger().info(
                f'Temperature: {self.temp}°C | Humidity: {self.hum}% | Lighting: {self.light} lux'
            )

            self.new_temp = False
            self.new_hum = False
            self.new_light = False


def main():
    rclpy.init()
    system_visuals = SystemVisualization()
    rclpy.spin(system_visuals)
    system_visuals.destroy_node()
    rclpy.shutdown()