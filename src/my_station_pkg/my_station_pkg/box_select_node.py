#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg._string import String
import re
import sys
from rclpy.executors import ExternalShutdownException


class box_select_node(Node):
    #constructor
    def __init__(self):
        """constructor"""
        super().__init__("box_select_node")
        #create the publisher
        self.publisher_ = self.create_publisher(String,
            "human_found",10)
        #create the subscriber
        self.subsriber_ = self.create_subscription(String,
            "result_boxes", self.select_box_callback,10)
        self.get_logger().info("box select node started")
    
    def select_box_callback(self, msg):
        """a callback to decide which box has the highest confidence"""
        self.get_logger().info("running select box node")
        try:
            print(msg.data)
            #seperate incoming data
            coord_string, conf_float = seperate_incoming_tuple(msg.data)
            Highest_conf_index = Find_confidence(conf_float)
            _target = String()
            _target.data = str(coord_string[Highest_conf_index])                                      
            self.publisher_.publish(_target)
            print("Target at: ",_target.data, " published")
        except:
            print("error")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = box_select_node()
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

def seperate_incoming_tuple(tuple_msg):
    """seperate the incomming message back into the 
    bounding box array and the confidence array"""
    lists = re.findall(r"\[.*?]",tuple_msg)                                                                 #split the data into two lists
    bbox_lists = []
    for i in range(len(lists)-1):                                                                  
        bbox_data = [list(map(float,x.strip("[]").split("''"))) for x in re.findall(r"[\d\.]+", lists[i])]   #Extract the bounding box data
        bbox_lists.append(bbox_data)
    conf_data = [float(x) for x in re.findall(r"[\d\.]+", lists[len(lists)-1])]                              #Extract the confidence data
    return bbox_lists, conf_data                                                                         #return both lists

def Find_confidence(confidence_list):
    """function used to find the highest 
    confidence in a list"""
    max_index = 0
    max_value = 0
    for i, value in enumerate(confidence_list):
        if value > max_value:
            max_value = value
            max_index = i
    return max_index