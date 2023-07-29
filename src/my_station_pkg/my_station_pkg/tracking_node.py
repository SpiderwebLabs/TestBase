#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg._string import String
import sys
from rclpy.executors import ExternalShutdownException
import re

class tracking_node(Node):
    def __init__(self):
        """constructor"""
        super().__init__("tracking_node")
        #create the publisher
        self.publisher_ = self.create_publisher(String,
            "error_value", 10)
        #create the subscriber
        self.subscriber_1_ = self.create_subscription(String,
            "human_found", self.tracking_callback, 10)
        self.get_logger().info(" node started")
    
    def tracking_callback(self, msg):
        """code to track the error between the centre 
        of the screen and the centre of the box"""
        self.get_logger().info("tracking_node running")
        try:
            my_list = find_center_box(msg.data)
            error_x =  my_list[0] - 0.5 
            error_y =  my_list[1] - 0.5
            error = [error_x,error_y]
            error_str = String()
            error_str.data = str(error)
            self.publisher_.publish(error_str)
            print("Error to target is: ",error_str.data)
            self.get_logger().info("Published data...")
        except:
            self.get_logger().info("error with data...")
        

def main(args = None):
    rclpy.init(args = args)
    try:
        node = tracking_node()
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

def find_center_box(xyxy = str):
    """take in raw data and make the 
    list with xywh normalised"""
     # Use regex to match and extract the numbers
    numbers = re.findall(r'\d + \.\d + ', xyxy)
    # Convert the extracted numbers to floats and store in a list
    return [float(num) for num in numbers]