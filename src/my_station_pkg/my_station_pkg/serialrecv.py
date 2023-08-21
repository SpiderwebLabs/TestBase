#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from more_interfaces.srv import String, Comms, Trigger
from more_interfaces.msg import Trials1
import serial
import threading
import time
import json
from time import sleep

SERIAL_PORT = '/dev/ttyUSB1'
BAUDRATE = 57600

class SerialReceiverNode(Node):
    def __init__(self):
        super().__init__('serialrecv')
        self.client_ = self.create_client(String, 'trigger_serial')
        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE)
        self.get_logger().info('Serial node started')
        self.sendmission_client = self.create_client(Comms, 'goto')
        self.land_client = self.create_client(Comms, 'land')
        self.return_client = self.create_client(Comms, 'returnback')
        self.missionclear_client = self.create_client(Comms, 'clear_mission')
        self.pausemission_client = self.create_client(Comms, 'pause_mission')
        self.Coordinates_client = self.create_client(Comms, 'get_coordinates')
        self.reroute_client = self.create_client(Comms, 'reroute')
        self.turn_right_client = self.create_client(Trigger, 'turn_right')
        self.turn_left_client = self.create_client(Trigger, 'turn_left')
        self.subscriber_telem = self.create_subscription(Trials1, 'status_update', self.callback)
        self.comms_request, self.trigger_request = Comms.Request(), Trigger.Request()
        # Start the background thread for reading from serial
        self.serial_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.serial_thread.start()

    def serial_read_loop(self):
        while True:
            if self.ser.in_waiting:
                print('Waiting for data....')
                print('Waiting for data....')
                data_bytes = self.ser.readline().decode().strip()
                # Decode the received bytes (assuming it's a string)
                received_data_string = data_bytes.decode()
                # Convert the JSON string back to a dictionary
                received_data = json.loads(received_data_string)
                # Now you can access the data elements as before
                received_type = received_data["type"]
                received_pos_longitude = received_data["data"]["x"]
                received_pos_latitude = received_data["data"]["y"]
                received_pos_altitude = received_data["data"]["z"]
                self.droneComms(received_type, received_pos_longitude, received_pos_latitude, received_pos_altitude)
            time.sleep(0.1)
  
                

    def data_callback(self, msg):
        data = {"is_in_air": msg.is_in_air,
                'armed': msg.armed,
                'battery': msg.battery_percentage,
                'y_coordinate': msg.pos_latitude,
                'x_coordinate': msg.pos_longitude,
                'z_coordinate': msg.pos_altitude,
                'vel_x': msg.velocity_x,
                'vel_y': msg.velocity_y,
                'vel_z': msg.velocity_z,
                'yaw': msg.yaw}
        try:
            self.send_data(data)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        else:
            self.get_logger().info(f"Message successfully sent")
        
    
    def send_data(self, data):
        # Convert the dictionary to a JSON string
        data_json_string = json.dumps(data)
        #sends the `data_json_string` over serial communication
        self.ser.write(data_json_string.encode())


    def droneComms(self, msg, lon, lat, alt):
        if msg == "Goto":
            self.comms_request.message = msg
            self.comms_request.pos_longitude = lon
            self.comms_request.pos_latitude = lat
            self.comms_request.pos_altitude = alt
            self.comms_request.trigger = True
            self.sendmission_client.call_async(self.comms_request)

        if msg == "Goback":
            self.comms_request.message = 'Goback'
            self.comms_request.trigger = True
            self.sendmission_client.call(self.comms_request)

        if msg == 'Clearmission':
            self.comms_request.message = 'Clearmission'
            self.comms_request.trigger = True
            self.sendmission_client.call_async(self.comms_request)

        if msg == 'reroute':
            self.comms_request.message = msg
            self.comms_request.pos_longitude = lon
            self.comms_request.pos_latitude = lat
            self.comms_request.pos_altitude = alt
            self.comms_request.trigger = True
            self.sendmission_client.call_async(self.comms_request)

        if msg == 'turn_right':
            self.trigger_request = True
            self.turn_right_client.call_async(self.trigger_request)

        if msg == 'turn_left':
            self.trigger_request = True
            self.turn_left_client.call_async(self.trigger_request)

        if msg == 'Land':
            self.comms_request.trigger = True
            self.comms_request.message = "Land"
            self.sendmission_client.call_async(self.comms_request)


def main(args=None):
    rclpy.init(args=args)
    node = SerialReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
