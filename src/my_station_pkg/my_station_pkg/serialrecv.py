#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from more_interfaces.srv import String
import serial
import threading
import time
from time import sleep

SERIAL_PORT = '/dev/ttyUSB1'
BAUDRATE = 57600

class SerialReceiverNode(Node):
    def __init__(self):
        super().__init__('serialrecv')
        self.client_ = self.create_client(String, 'trigger_serial')
        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE)
        self.get_logger().info('Serial node started')
        while True:
            while self.ser.in_waiting:
                print('Waiting for data....')
                data = self.ser.readline().decode().strip()
                
                sleep(0.5)
            


def main(args=None):
    rclpy.init(args=args)
    node = SerialReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
