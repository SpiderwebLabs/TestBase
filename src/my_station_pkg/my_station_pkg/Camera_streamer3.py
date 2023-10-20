#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoViewerNode(Node):
    def __init__(self):
        super().__init__('Camera_streamer3')
        self.publisher_ = self.create_publisher(Image, 'camera_image3', 10)  # Create an Image publisher
        self.ip_address = 'http://0.0.0.0:8080'  
        self.video_url = f'{self.ip_address}/video_feed'
        self.bridge = CvBridge()

    def run(self):
        cap = cv2.VideoCapture(self.video_url)
        if not cap.isOpened():
            self.get_logger().error("Error: Could not open video stream.")
            return

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Publish the frame to the topic
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    video_viewer_node = VideoViewerNode()
    video_viewer_node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
