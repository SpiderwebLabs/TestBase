import asyncio

import websockets
import os
import rclpy
import future
import json
from decimal import Decimal
from sensor_msgs.msg import Image
from rclpy.node import Node
from more_interfaces.srv import Alert
import sys
from rclpy.executors import ExternalShutdownException
import cv2
from cv_bridge import CvBridge

class MyNode(Node):
    def __init__(self):
        super().__init__('ip_cam')
        self.subscription_ = self.create_subscription(Image, 'image_classifier', self.image_classifier_callback, 10)
        '''create the websocket server'''  
        self.server = websockets.serve(
            self.handle_websocket_request, '10.242.132.251', 8000)  # create a websocket server to receive coordinates from the api
        self.get_logger().info("Websocket server started")
        self.person_name = ""
        self.bridge = CvBridge()
        self.frame_count = 0
        self.track_id = None
        self.start_saving_images = False
        self.saved_frame_count = 0  
        self.get_trackid = False
        

    async def handle_websocket_request(self, websocket, path):
        '''Wait for a message to be received over the websocket'''
        message = await websocket.recv()
        message = json.loads(message)
        self.person_name = message['person_name']
        self.start_saving_images = True  # Start saving images when a request is received
        self.saved_frame_count = 0  # Reset the saved frame count
        print(message)
    
        # '''Wait for the response from the server node'''
        await websocket.send("response message")  # send a response message to the api


    def image_classifier_callback(self, msg):
        if self.get_trackid:
            self.get_logger().info(f"Received track id")
            self.track_id = msg.frame_id
            self.get_trackid = False
        if not self.start_saving_images:
            return
        elif self.saved_frame_count < 10:  # Save up to 10 frames
            if not os.path.exists(os.path.join("data", self.person_name)):
                os.mkdir(os.path.join("data", self.person_name))
                self.get_logger().info(f"Created folder {os.path.join('data', self.person_name)}")
            cv2.imwrite(os.path.join("data", self.person_name, f"{self.frame_count}_{self.track_id}.jpg"), self.bridge.imgmsg_to_cv2(msg, "bgr8"))
            self.frame_count += 1
            self.saved_frame_count += 1
            if self.saved_frame_count == 10:
                self.get_logger().info(f"Saved 10 frames for {self.person_name}. Stopping image saving.")
                self.start_saving_images = False


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
