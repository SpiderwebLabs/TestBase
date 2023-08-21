import rclpy
from rclpy.node import Node
from more_interfaces.srv import Arduino1
from std_msgs.msg import String
import threading
import time

class ServerNode(Node):
    def __init__(self):
        super().__init__('server_node')
        self.name = 0
        self.server_ = self.create_service(Arduino1, 'testcomms', self.servercallback)

    def servercallback(self, request, response):
        req = request.message
        self.update_name(req)
        response.outcome = True
        return response
    
    def update_name(self, input):
        if input == 'start':
            for i in range(2, 100, 2):
                self.name = i
                print(i)
                time.sleep(0.1)

        else:
            self.name = 300


class PubNode(Node):
    def __init__(self, server_node):
        super().__init__('pub_node')
        self.server_node = server_node  # Store reference to the server node
        self.pub_ = self.create_publisher(String, 'pubtopic', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = str(self.server_node.name)  # Access attribute of the server node
        self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    server_node = ServerNode()
    pub_node = PubNode(server_node)  # Pass the server node instance
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(server_node)
    executor.add_node(pub_node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        server_node.destroy_node()
        pub_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
