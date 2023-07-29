import rclpy
from rclpy.node import Node
from more_interfaces.srv import Timer


class TimerServerNode(Node):

    def __init__(self):
        super().__init__('timer_node')
        self.timer = None
        self.completed = False
        self.timer_expired_client = self.create_client(Timer, 'timer_complete')
        while not self.timer_expired_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Timer complete service not available, waiting...')

        self.timer_control_server = self.create_service(Timer, 'timer_control', self.timer_control_callback)

    def timer_control_callback(self, request, response):
        if request.trigger and not self.timer:
            self.timer = self.create_timer(request.time, self.timer_callback)
            self.completed = False
            self.get_logger().info(f'Timer started for {request.time}seconds')
            response.success = True
            response.message = 'Timer_Started'
        elif not request.trigger and self.timer:
            self.timer.cancel()
            self.timer = None
            self.completed = False
            self.get_logger().info('Timer stopped and reset to 0')
            response.success = True
            response.message = 'Timer_Reset'
        else:
            response.success = False
            response.message = 'Invalid_Timer_Request'

        return response

    def timer_callback(self):
        self.completed = True
        self.timer.cancel()
        self.timer = None
        self.get_logger().info('Timer completed')
        self.make_request()

    def make_request(self):
        if self.completed:
            request = Timer.Request()
            request.trigger = True
            self.timer_expired_client.call_async(request)
            self.completed = False


        

def main(args=None):
    rclpy.init(args=args)
    node = TimerServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
