#!/usr/bin/env python3
from more_interfaces.srv import Error
import rclpy
from rclpy.node import Node
from std_msgs.msg._string import String
import re
import sys
from rclpy.executors import ExternalShutdownException


class error_tracking_node(Node):

    def __init__(self):
        """constructor"""
        super().__init__("error_tracking_node")
        self.FOV_ = 80                                  #The FOV is standard and depends on the camera used
        self.errorVal = 0
        self._srv = self.create_service(Error, 'error_tracking', self.track_error)
        self.subscriber_ = self.create_subscription(String, 'error_value', self.subscriber_callback, 10)
        self.get_logger().info("error tracking node started")

    def subscriber_callback(self, msg):
        """PID back call function"""

        #Convert msg to normalized x
        err = get_xy(msg.data)
        self.get_logger().info(f'{msg.data}')
        #Set self.errorVal to normalized x * self.FOV
        self.errorVal = err[0] * self.FOV_
        

    def track_error(self, request, response):
        """a server request callback used to recieve the current yaw
        and output the desired  yaw"""
        try:
            desired_yaw = request.current_yaw + self.errorVal
            response.desired_yaw = float(desired_yaw)
            self.errorVal = 0
            return response
        except:
            self.get_logger().info(f'you lose')



def main(args = None):
    rclpy.init(args=args)
    try:
        node = error_tracking_node()
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

def get_xy(xy):
    """take in raw data and make the 
    list with xywh normalised"""
     # Use regex to match and extract the numbers
    numbers = re.findall(r'\d+\.\d+',xy)
    # Convert the extracted numbers to floats and store in a list
    return [float(num) for num in numbers]