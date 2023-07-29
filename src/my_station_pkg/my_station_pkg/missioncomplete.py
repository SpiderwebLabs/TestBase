#!/usr/bin/env python3
from more_interfaces.srv import Comms, Trigger
from more_interfaces.msg import Trials1
import rclpy
from rclpy.node import Node
from decimal import Decimal
from geopy import distance
import rclpy.qos


class SlaveNode(Node):
    def __init__(self):
        super().__init__('missioncomplete')
        self.getmisioncod_server = self.create_service(Comms, 'get_coordinates', self.getcoordinates_callback)
        self.getcurrentcod_subcriber = self.create_subscription(Trials1, 'status_update', self.getcurrentcod_callback, 10)
        #self.publish_missioncomplete = self.create_publisher(Trials1, 'mission_complete')
        self.missioncompleteclient = self.create_client(Trigger, 'mission_complete')

        self.flight_latitude = None
        self.flight_longitude = None
        self.latitude = None
        self.longitude = None
        self.mission_complete = False
        self.last_mission_complete = False

    def getcurrentcod_callback(self, msg):
        self.latitude = msg.pos_latitude
        self.longitude = msg.pos_longitude
        #self.get_logger().info(f"cur_longitude: {self.longitude}, cur_latitude: {self.latitude}")
        self.mission_complete_check()
    
    def getcoordinates_callback(self, request, response):
        self.flight_latitude = Decimal(request.pos_latitude)
        self.flight_longitude = Decimal(request.pos_longitude)
        #self.get_logger().info(f"mision_longitude: {self.flight_longitude}, mission_latitude: {self.flight_latitude}")
        response.connect_success = True
        return response
    
    def mission_complete_check(self):
        '''Function to check if drone has reached the target location'''
        cur_pos = (self.latitude, self.longitude)
        target_pos = (self.flight_latitude, self.flight_longitude)
        mission_complete_recv =  distance.distance(cur_pos, target_pos).m < 2.0
        #self.get_logger().info(f"Distance: {distance.distance(cur_pos, target_pos).m }")
        if mission_complete_recv and not self.last_mission_complete:
            self.get_logger().info('Mission complete')
            msg = Trigger.Request()
            msg.trigger = True
            self.missioncompleteclient.call_async(msg)
        self.last_mission_complete = mission_complete_recv

def main(args=None):
    rclpy.init(args=args)
    node = SlaveNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
