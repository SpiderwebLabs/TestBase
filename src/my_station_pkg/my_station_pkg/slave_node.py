#!/usr/bin/env python3
from std_msgs.msg._string import String
import rclpy
from rclpy.node import Node

class SlaveNode(Node):
    def __init__(self):
        super().__init__('slave_node')
        self.publisher_ = self.create_publisher(String, "error_value", 10)  # Change topic name and message type if needed
        self.timer_period = 1.0  # Set the publishing rate (in seconds)
        self.timer = self.create_timer(self.timer_period, self.publish_message)

    def publish_message(self):
        error = [1.0, 2.0]
        error_str = String()
        error_str.data = str(error)
        self.publisher_.publish(error_str)
        
    
def main(args=None):
    rclpy.init(args=args)
    node = SlaveNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()














# import rclpy
# from rclpy.node import Node
# from mavsdk import System
# from mavsdk.offboard import (OffboardError, PositionNedYaw)
# from more_interfaces.srv import Comms
# import asyncio

# class ControlServerNode(Node):
#     def __init__(self):
#         super().__init__('slave_node')
#         self.drone = System()

#         self.arm_service = self.create_service(Comms, 'arm', self.arm_callback)
#         self.takeoff_service = self.create_service(Comms, 'takeoff', self.takeoff_callback)
#         self.goto_service = self.create_service(Comms, 'goto', self.goto_callback)
#         self.land_service = self.create_service(Comms, 'land', self.land_callback)

#         # Connect to the drone
#         self.loop = asyncio.get_event_loop()
#         self.loop.run_until_complete(self.connect_to_drone())

#     async def connect_to_drone(self):
#         self.get_logger().info('Waiting for the drone to connect...')

#         try:
#             await self._connect_to_drone()
#         except Exception as e:
#             self.get_logger().error(f"Drone connection failed: {str(e)}")

#         self.get_logger().info('Drone connected')

#     async def _connect_to_drone(self):
#         await self.drone.connect(system_address="udp://:14540")

#     async def arm_callback(self, request, response):
#         self.loop.run_until_complete(self.perform_arm())
#         response.connect_success = True
#         return response
    
#     async def perform_arm(self):
#         await self.drone.action.arm()
#         self.get_logger().info('Drone armed')


#     async def takeoff_callback(self, request, response): 
#         self.loop.run_until_complete(self.perform_takeoff())
#         response.connect_success = True
#         return response
    
#     async def perform_takeoff(self):
#         await self.drone.action.takeoff()
#         self.get_logger().info('Drone taking off')

#     def goto_callback(self, request, response):
#         if self.armed:
#             self.get_logger().info(f'Drone going to {request.x}, {request.y}, {request.z}, {request.yaw}')
#         else:
#             self.get_logger().info('Drone is not armed')

#         response.connect_success = True
#         return response

#     def land_callback(self, request, response):
#         if self.armed:
#             self.drone.action.land()
#             self.get_logger().info('Drone landing')
#         else:
#             self.get_logger().info('Drone is not armed')

#         response.connect_success = True
#         return response


# def main(args=None):
#     rclpy.init(args=args)
#     control_server = ControlServerNode()
#     rclpy.spin(control_server)
#     control_server.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
