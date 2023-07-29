#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from more_interfaces.srv import String, Comms
import serial
import threading
import time
from time import sleep

SERIAL_PORT = '/dev/ttyUSB1'
BAUDRATE = 57600

class SerialTransmitterNode(Node):
    def __init__(self):
        super().__init__('serial')
        self.goto = self.create_service(Comms, "goto", self.goto_callback)
        self.land = self.create_service(Comms, "land, self.land_callback")
        self.returnback = self.create_service(Comms, "returnback", self.returnback_callback)
        self.clearmission = self.create_service(Comms, 'clear_mission',self.clearmission)
        self.pausemission = self.create_service(Comms, "reroute", self.reroute_callback)
        self.turnright = self.create_service(Comms, "turn_right", self.turnright_callback)
        self.turnleft = self.create_service(Comms, "turn_left", self.turnleft_callback)
        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE)
        self.lock = threading.Lock()
        self.get_logger().info('Serial node started')

    def serial_callback(self, request, response):
        print("Request arrived")
        data = request.message
        with self.lock:
            try:
                self.send_data(data)
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                response.outcome = False
            else:
                self.get_logger().info(f"Message '{data}' successfully sent")
                response.outcome = True
        return response 
    
    def send_data(self, data):
        self.ser.write(data.encode())

def main(args=None):
    rclpy.init(args=args)
    node = SerialTransmitterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
