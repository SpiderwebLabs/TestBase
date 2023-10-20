#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg._string import String
import cv2
from ultralytics import YOLO
import cv2
import numpy as np

class AINode(Node):
    def __init__(self):
        super().__init__('ip_cam_ai')
        self.subscription1 = self.create_subscription(
            Image, 'camera_image', self.image_callback, 10)
        self.subscription2 = self.create_subscription(
            Image, 'camera_image2', self.image_callback2, 10)
        self.subscription3 = self.create_subscription(
            Image, 'camera_image3', self.image_callback3, 10)
        self.publisher_ = self.create_publisher(String,
            "human_detected",10)
        #instantiate the model object
        self.model_ = YOLO("yolov8n.pt")
        #set up communication between ros2 and cv2
        self.bridge = CvBridge()
        self.processed_frame_camera1 = None
        self.processed_frame_camera2 = None
        self.processed_frame_camera3 = None

    def image_callback(self, msg):
        self.processed_frame_camera1 = self.process_image(msg, "camera1")
        cv2.imshow('AI Viewer - Camera 1', self.processed_frame_camera1)
        self.handle_keypress()

    def image_callback2(self, msg):
        self.processed_frame_camera2 = self.process_image(msg, 'camera2')
        cv2.imshow('AI Viewer - Camera 2', self.processed_frame_camera2)
        self.handle_keypress()

    def image_callback3(self, msg):
        self.processed_frame_camera3 = self.process_image(msg, 'camera3')
        cv2.imshow('AI Viewer - Camera 3', self.processed_frame_camera3)
        self.handle_keypress()

    def process_image(self, msg, camera_ID):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model_.predict(source=frame)                    #run YOLOv8 on the incomming frame
            #initialise two arrays
            coord_str = []                                              
            conf_float = []
            #obtain the boxes subclass from the results class
            boxes_ = results[0].boxes
            human_detetced = False                                                      #set the initial flag to false
            for detected in boxes_:
                if detected.cls == 0:
                    #if a human has been detected
                    coord_str.append(self.make_coord_nice(str(detected.xywhn)))               #get the coordinates only
                    conf_float.append(self.make_conf_nice(str(detected.conf)))               #get the confidence only
                    human_detected = True  
                    if human_detected:
                        pub_tuple = ("$",coord_str,"&","+",conf_float,"-")                                      #make the two arrays into a single tuple
                        msg = String()
                        msg.data = camera_ID                                             #make the tuple into a string
                        self.publisher_.publish(msg) 

        except Exception as e:
            self.get_logger().error("Error converting Image message: %s" % str(e))
            return frame  
        processed_frame = frame  
        return processed_frame
   
    def make_coord_nice(self, coordinate = str):
        """a function to annotate the 
        string recieved from boxes.xywhn to 
        a coordinate"""
        start = coordinate.find("[") 
        end = coordinate.find("]") +1
        return coordinate[start+1:end]

    def make_conf_nice(self, confidence = str):
        """a function to annotate the 
        string recieved from boxes.conf to 
        a confidence"""
        start = confidence.find("[") 
        end = confidence.find("]") 
        return float(confidence[start+1:end])*100

    def handle_keypress(self):
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    ai_node = AINode()
    rclpy.spin(ai_node)
    ai_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

