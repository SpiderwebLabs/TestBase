#!/usr/bin/env python3

#imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg._compressed_image import CompressedImage
from std_msgs.msg._string import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import sys
from rclpy.executors import ExternalShutdownException


class ai_node(Node):
    #constructor
    def __init__(self):
        """constructor"""
        super().__init__("ai_node")
        #set up communication between ros2 and cv2
        self.bridge_ = CvBridge()
        #instantiate the model object
        self.model_ = YOLO("yolov8n.pt")
        #create publisher 
        self.publisher_ = self.create_publisher(String,
            "result_boxes",10)
        #create subscriber
        self.subscriber_ = self.create_subscription(CompressedImage,
            "video_data",self.ai_node_callback,10)
        self.get_logger().info("AI node started")
        

    def ai_node_callback(self,img):
        """ai_node"""
        self.get_logger().info("AI running on image data...")
        try:
            #convert the image back to a readable cv file
            frame = self.bridge_.compressed_imgmsg_to_cv2(img,"passthrough")            #change the format of the frame 
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
                    coord_str.append(make_coord_nice(str(detected.xywhn)))               #get the coordinates only
                    conf_float.append(make_conf_nice(str(detected.conf)))               #get the confidence only
                    human_detetced = True                                               #set the flag equal to true
            #publish if a human has been detected
            if human_detetced:
                pub_tuple = ("$",coord_str,"&","+",conf_float,"-")                                      #make the two arrays into a single tuple
                msg = String()
                msg.data = str(pub_tuple)                                               #make the tuple into a string
                self.publisher_.publish(msg)                                            #publish the tuple
                print("published")
            print("processed")
        except :
            print("error")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ai_node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()


def make_coord_nice(coordinate = str):
    """a function to annotate the 
    string recieved from boxes.xywhn to 
    a coordinate"""
    start = coordinate.find("[") 
    end = coordinate.find("]") +1
    return coordinate[start+1:end]

def make_conf_nice(confidence = str):
    """a function to annotate the 
    string recieved from boxes.conf to 
    a confidence"""
    start = confidence.find("[") 
    end = confidence.find("]") 
    return float(confidence[start+1:end])*100