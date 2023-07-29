import websocket
import rclpy
import asyncio
from rclpy.node import Node
from more_interfaces.srv import String


class BatteryChanger(Node):
    def __init__(self):
        super().__init__("BatteryHolder")
        # WebSocket server URL
        self.websocket_server = 'ws://172.20.10.7/ws'  # Replace <ESP32_IP_ADDRESS> with the actual IP address
        # Creates a server that receives commands to replace
        self.battery_server = self.create_service(String, "battery_holder", self.callback_commands)
        # Connect to the WebSocket server
        #self.ws = websocket.create_connection(self.websocket_server)
        self.loop = asyncio.get_event_loop()
        self.connect = False
    
    async def connect_callback(self):
        self.get_logger().info('Connecting to websocket...')
        try:
            await self.connect_websocket()
            self.connect = True
        except Exception as e:
            self.get_logger().error(f"Connecting to websocket: {str(e)}")
            self.connect = False
        self.get_logger().info('Websocket connected')
        
    async def connect_websocket(self):
        self.ws = websocket.create_connection(self.websocket_server)

    # Send commands to the server
    def callback_commands(self, request, response):
        try:
            self.loop.run_until_complete(self.connect_callback())
            command = request.message
            self.ws.send(command)
            response_msg = self.ws.recv()
            self.get_logger().info(f"Response from server: {response_msg}")

        except:
            self.get_logger().info("Failed to connect")
            self.connect = False
        self.get_logger().info(f"{self.connect}")
        if self.connect:
            response.outcome = True
        else:
            response.outcome = False

        return response


def main(args=None):
    '''Create a ROS2 node, spin it'''
    rclpy.init(args=args)
    node = BatteryChanger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
