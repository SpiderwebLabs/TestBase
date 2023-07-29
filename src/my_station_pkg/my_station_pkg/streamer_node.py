#!/usr/bin/env python3
#imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg._compressed_image import CompressedImage
from more_interfaces import String
import cv2
from cv_bridge import CvBridge
import time
import sys
from rclpy.executors import ExternalShutdownException


class streamer_node(Node):
    #initialise the node
    def __init__(self):
        """constructor"""
        #define the constructor
        super().__init__("streamer_node")
        #declare the communication between ros and CV2
        self.bridge_ = CvBridge()
        self.publisher_ = self.create_publisher(CompressedImage, "video_data", 10)
        self.get_logger().info("streamer node has started")
        self.start_stop_streamer = self.create_service(String, "start_stop_streamer", self.startcallback)
        self.start_flag = False

    def startcallback(self, request, response):
        if request.message == "start":
            self.start_flag = True
        else:
            self.start_flag = False
            
        response.outcome
        return response

    
    def Video_pub(self):
        """a function used to proccess and publish a video"""
        #read the video
        self.capture_ = cv2.VideoCapture(0)
        #check if the video can be opened
        if not self.capture_.isOpened():
            self.get_logger().info("Publisher unable to open image")
            exit()
        while (True):
            time.sleep(0.1)
            #capture the current frame
            success,frame = self.capture_.read()
            #check if the current frame was read correctly
            if not success:
                self.get_logger().info("Publisher unable to recieve frame (possibly end of stream)")
                break
            #publish the frames
            msg = self.bridge_.cv2_to_compressed_imgmsg(frame,"jpg")
            self.publisher_.publish(msg)
    
        self.capture_.release()
            
#the main function
def main(args = None):
    """this function will run""" 
    rclpy.init(args = args)
    try:
        node = streamer_node()
        node.Video_pub()
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
