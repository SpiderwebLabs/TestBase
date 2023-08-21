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

SERIAL_PORT = '/dev/ttyRadiotelem'
BAUDRATE = 57600

class SerialTransmitterNode(Node):
    def __init__(self):
        super().__init__('serial')
        self.goto = self.create_service(Comms, "goto", self.goto_callback)
        self.land = self.create_service(Comms, "land", self.land_callback)
        self.returnback = self.create_service(Comms, "returnback", self.returnback_callback)
        self.clearmission = self.create_service(Comms, 'clear_mission',self.clearmission)
        self.pausemission = self.create_service(Comms, "reroute", self.reroute_callback)
        self.turnright = self.create_service(Trigger, "turn_right", self.turnright_callback)
        self.turnleft = self.create_service(Trigger, "turn_left", self.turnleft_callback)
        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE)
        self.lock = threading.Lock()
        self.get_logger().info('Serial node started')
        
  
    def goto_callback(self, request, response):
        data = {"type": request.message,
                "data":{"x": request.pos_longitude,
                "y": request.pos_latitude,
                "z": request.pos_altitude}}
        
        with self.lock:
            try:
                self.send_data(data)
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                response.outcome = False
            else:
                self.get_logger().info(f"Message '{data}' successfully sent")
                response.connect_success = True
        return response 

    def land_callback(self, request, response):
        data = {'type': request.message,
                'data': { 'x': 0,
                         'y': 0,
                         'z': 0
                }}
        with self.lock:
            try:
                self.send_data(data)
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                response.connect_success = False
            else:
                self.get_logger().info(f"Message '{data}' successfully sent")
                response.connect_success = True
        return response 

    def returnback_callback(self, request, response):
        data = {'type': request.message,
                'data': { 'x': 0,
                         'y': 0,
                         'z': 0
                }}
        with self.lock:
            try:
                self.send_data(data)
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                response.outcome = False
            else:
                self.get_logger().info(f"Message '{data}' successfully sent")
                response.connect_success = True
        return response 

    def clearmission(self, request, response):
        data = {'type': request.message,
                'data': { 'x': 0,
                         'y': 0,
                         'z': 0
                }}
        with self.lock:
            try:
                self.send_data(data)
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                response.outcome = False
            else:
                self.get_logger().info(f"Message '{data}' successfully sent")
                response.connect_success = True
        return response 

    def reroute_callback(self, request, response):
        data = {"type": request.message,
                "data":{"x": request.pos_longitude,
                "y": request.pos_latitude,
                "z": request.pos_altitude}}
        
        with self.lock:
            try:
                self.send_data(data)
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                response.outcome = False
            else:
                self.get_logger().info(f"Message '{data}' successfully sent")
                response.connect_success = True
        return response 
    
    def turnright_callback(self, request, response):
        data = {'type': 'turn_right',
                'data': { 'x': 0,
                         'y': 0,
                         'z': 0
                }}
        with self.lock:
            try:
                self.send_data(data)
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                response.success = False
            else:
                self.get_logger().info(f"Message '{data}' successfully sent")
                response.success = True
        return response 

    def turnleft_callback(self, request, response):
        data = {'type': 'turn_left',
                'data': { 'x': 0,
                         'y': 0,
                         'z': 0
                }}
        with self.lock:
            try:
                self.send_data(data)
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                response.success = False
            else:
                self.get_logger().info(f"Message '{data}' successfully sent")
                response.success = True
        return response  
    
    # def send_data(self, data):
    #     # Convert the dictionary to a JSON string
    #     data_json_string = json.dumps(data) + '\n'
    #     # Convert the JSON string to a bytearray
    #     data_bytes = bytearray(data_json_string, 'utf-8')
    #     # Send the bytearray over serial communication
    #     self.ser.write(data_bytes)

    def send_data(self, data):
        try:
            # Convert the dictionary to a JSON string
            data_json_string = json.dumps(data) + '\n'
            # Convert the JSON string to a bytearray
            data_bytes = bytearray(data_json_string, 'utf-8')
            # Send the bytearray over serial communication
            self.ser.write(data_bytes)
        except Exception as e:
            self.get_logger().error(f"Error sending data: {e}")

class SerialRead(Node):
    def __init__(self, serial_trans):
        super().__init__('pub_node')
        self.serial_trans = serial_trans
        self.telempub = self.create_publisher(Trials1, 'status_update', 10)
        self.msg = Trials1()

    def serial_read_loop(self):
        while True:
            if self.serial_trans.ser.in_waiting:
                print('Waiting for data....')
                try:
                    data_bytes = self.serial_trans.ser.readline().decode().strip()
                    print(data_bytes)

                    if data_bytes:  # Check if the received data is not empty
                        received_data = json.loads(data_bytes)
                        # Rest of your code to process the received_data
                        self.msg.is_in_air = received_data["is_in_air"]
                        self.msg.armed = received_data['armed']
                        self.msg.battery_percentage = received_data['battery']
                        self.msg.pos_longitude = received_data['x_coordinate']
                        self.msg.pos_latitude = received_data['y_coordinate']
                        self.msg.pos_altitude = received_data['z_coordinate']
                        self.msg.velocity_x = received_data['vel_x']
                        self.msg.velocity_y = received_data['vel_y']
                        self.msg.velocity_z = received_data['vel_z']
                        self.msg.current_yaw = received_data['yaw']
                        self.msg.landed_state = received_data['landed_state']
                        
                        self.telempub.publish(self.msg)
                        time.sleep(0.3)
                    else:
                        self.get_logger().warning("Received empty data, skipping...")
                except json.JSONDecodeError as e:
                    self.get_logger().error(f'JSONDecodeError: {e}, Data: {data_bytes}')
                except Exception as e:
                    self.get_logger().error(f'Error is {e}')

        



def main(args=None):
    rclpy.init(args=args)
    serial_trans = SerialTransmitterNode()
    serial_read = SerialRead(serial_trans)
     # Create a thread to run the serial_read_loop function
    read_thread = threading.Thread(target=serial_read.serial_read_loop)
    read_thread.daemon = True  # Set the thread as a daemon so it exits when the main program exits
    read_thread.start()  # Start the thread

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(serial_trans)
    executor.add_node(serial_read)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        serial_trans.destroy_node()
        serial_read.destroy_node()
        rclpy.shutdown()



    rclpy.shutdown()

if __name__ == '__main__':
    main()
