import asyncio

import websockets

import rclpy
import future
import json
from decimal import Decimal

from rclpy.node import Node
from more_interfaces.srv import Alert
import sys
from rclpy.executors import ExternalShutdownException

class MyNode(Node):
    def __init__(self):
        super().__init__('message_receiver_node')
        '''Create a ROS2 client to send requests to the server node'''
        self.client = self.create_client(
            Alert, "set_drone_action")  # create a client for sending messages to the sys_controller_node
        '''create the websocket server'''  
        self.server = websockets.serve(
            self.handle_websocket_request, '10.242.132.251', 8000)  # create a websocket server to receive coordinates from the api
        self.get_logger().info("Websocket server started")

    async def handle_websocket_request(self, websocket, path):
        '''Wait for a message to be received over the websocket'''
        message = await websocket.recv()  # wait for a message to be received over the websocket
        message = json.loads(message)  # convert the message to a json object
        

        '''Parse the message'''
        type = message['type']
        if type == 'start_mission':
            type = 'start'
        elif type == 'start_mission_patrol':
            type= 'patrol'
        self.get_logger().info(f"{type}")
        waypoints = message['waypoints']
        waypoints_list = [coord for point in waypoints for coord in (point["x"], point["y"])]
        mission_id = message['mission_id']

        self.get_logger().info(f"{waypoints_list}")

        '''Create a request message and send it to the server node'''
        self.request = Alert.Request()
        self.request.mission_id = mission_id
        self.request.waypoints = waypoints_list
        self.request.execute_action = type # action to be done by system controler node to either stsrt mission,sbort mission,take manual control,turn_left,turn_right
        self.request.alert_trigger = True
        # '''Wait for the response from the server node'''
        await websocket.send("response message")  # send a response message to the api

        self.future = self.client.call_async(self.request)
        # '''Wait for the response from the server node'''
        self.get_logger().info("Waiting for response from server node")
        # wait for the response from the server node
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("Response received :" + str(self.future.result().success_trigger))
        return self.future.result()


def main(args = None):
    '''Create a ROS2 node, spin it'''
    rclpy.init(args = args)
    node = MyNode()
    asyncio.get_event_loop().run_until_complete(node.server)
    asyncio.get_event_loop().run_forever()
    node.destroy_node()
    rclpy.shutdown()
   

if __name__ == '__main__':
    main()
