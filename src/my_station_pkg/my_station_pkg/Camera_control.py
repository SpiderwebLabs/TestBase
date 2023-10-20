#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg._string import String
import cv2

class CameraControlNode(Node):
    def __init__(self):
        super().__init__('Camera_control')
        self.subs = self.create_subscription(String, "human_detected", self.camera_control_callback, 10)

    def camera_control_callback(self, msg):
        camera_id = msg.data
        #human_detected = msg.human_detected
        print(camera_id)


def main(args=None):
    rclpy.init(args=args)
    camera= CameraControlNode()
    rclpy.spin(camera)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
