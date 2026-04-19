import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class LockService(Node):

    def __init__(self):
        super().__init__('lock_service')
        self.srv = self.create_service(SetBool, '/home/door/lock', self.lock_callback)
        self.door = 'unlocked'
    def lock_callback(self, request, response):
        if request.data == True:
            self.door = 'locked'
            response.success = True
            response.message = 'The door is locked.'
        if request.data == False:
            self.door = 'unlocked'
            response.success = True
            response.message = 'The door is unlocked.'
        return response
    
def main():
    rclpy.init()

    lock_service = LockService()
    rclpy.spin(lock_service)

    rclpy.shutdown()
