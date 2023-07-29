import rclpy
from rclpy.node import Node
from more_interfaces.srv import Timer


class TimerServer(Node):
    def __init__(self):
        super().__init__('timer_server')
        self.timer_complete_service = self.create_service(Timer, 'timer_complete', self.server_callback)

    def server_callback(self, request, response):
        self.get_logger().info(f"Timer completed: {request.trigger}")
        response.success = True
        response.message = 'Timer complete response'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TimerServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


