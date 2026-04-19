import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

class Alarm(Node):

    def __init__(self):
        super().__init__('alarm_service')
        self.create_subscription(Float64, '/home/temperature', self.temperature_callback, 10)
        self.srv = self.create_service(SetBool, '/home/alarm/activate', self.alarm_callback)
        self.alarm = False
    def alarm_callback(self, request, response):
        if request.data == True:
            self.alarm = True
            response.success = True
        if request.data == False:
            self.alarm = False
            response.success = True
        return response
    def temperature_callback(self, msg):
        if self.alarm == True and (msg.data < 15.0 or msg.data >35.0):
            self.get_logger().warn(f'Alarm active: Temperature out of range (15-35°C)! Current: {msg.data}°C ')

def main():
    rclpy.init()

    alarm = Alarm()
    rclpy.spin(alarm)

    rclpy.shutdown()