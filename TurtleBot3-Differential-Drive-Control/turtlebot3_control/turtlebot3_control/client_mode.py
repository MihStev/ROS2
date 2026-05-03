import rclpy

from rclpy.node import Node

from turtlebot3_interfaces.srv import SetMode

class ModeClientNode(Node):
    def __init__(self):
        super().__init__('mode_client_node')
        self.cli = self.create_client(SetMode, 'set_mode')


        while not self.cli.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = SetMode.Request()

    def send_request(self, mode, cmd_val, x_val, y_val):
        # Populate the request with the desired mode and parameters
        self.req.mode = mode
        self.req.manual_cmd = cmd_val
        self.req.target_x = x_val
        self.req.target_y = y_val

        # Send the request and wait for the response
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)

        return future.result()
    
def main(args=None):
    rclpy.init(args=args)
    client = ModeClientNode()


    while rclpy.ok():
        print("\n" + "="*30)
        print("Control Modes:")
        print("0: IDLE")
        print("1: MANUAL")
        print("2: AUTO")
        print("="*30)
        mode_input = input("Enter mode (0, 1, 2): ")

        try:
            mode = int(mode_input)
            if mode not in [0, 1, 2]:
                print("Invalid mode. Please enter 0, 1, or 2.")
                continue
        except ValueError:
            print("Invalid input. Please enter a number (0, 1, or 2).")
            continue

        cmd = ""
        x = 0.0
        y = 0.0

        if mode == 1:  # MANUAL mode
            cmd = input("Enter manual command (w/s/a/d for forward/backward/left/right): ")
        elif mode == 2:  # AUTO mode
            try:
                x = float(input("Enter target X coordinate: "))
                y = float(input("Enter target Y coordinate: "))
            except ValueError:
                print("Invalid input. Please enter numeric values for coordinates.")
                continue
        response = client.send_request(mode, cmd, x, y)

        if response.success:
            print(f"Mode set successfully: {response.message}")

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  
