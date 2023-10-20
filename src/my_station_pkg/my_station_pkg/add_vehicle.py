#!/usr/bin/env python3
import requests
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from more_interfaces.msg import Carplate
from cv_bridge import CvBridge

import time

farm_id_default = "7d3ab54f0c584bf3a0f8fdffe8e3c151"  # Zurich farm
username_default = "Zurich"
password_default = "12345"


class CameraControlNode(Node):
    def __init__(self):
        super().__init__('add_vehicle')
        self.access_token = None
        self.refresh_token = None
        self.declare_parameter("farm_id", farm_id_default)
        self.declare_parameter("username", username_default)
        self.declare_parameter("password", password_default)
        self.sub_ = self.create_subscription(Carplate, "add_vehicle", self.vehicle_control_callback, 10)
        login_url = 'http://102.37.220.214:8080/auth/login'

        self.farm_id_param = self.get_parameter("farm_id").get_parameter_value().string_value
        self.username_param = self.get_parameter("username").get_parameter_value().string_value
        self.password_param = self.get_parameter("password").get_parameter_value().string_value
        login_data = {'username': self.username_param, 'password': self.password_param}
        self.response = requests.post(login_url, json=login_data)
        self.get_logger().info("api node started")

        if self.response.status_code == 200:
            '''If the login was successful, save the access token'''
            self.access_token = self.response.json()['access_token']
            self.refresh_token = self.response.json()['refresh_token']

        else:
            '''If the login failed, log the error'''
            self.get_logger().error('Error logging in: {}'.format(self.response.text))

    def vehicle_control_callback(self, msg):
        vehicle_type = str(msg.type_vehicle)
        plate_info = msg.plate_info
        camera_id   = msg.camera_id
        self.vehicle_ctrl(vehicle_type, plate_info, camera_id)

    def vehicle_ctrl(self, type_vehicle, plate_info, camera_id):
        vehicle_api_url = f"http://102.37.220.214:8080/{self.farm_id_param}/assets/vehicles"
        headers = {'Authorization': 'Bearer {}'.format(self.access_token)}
        api_message = {
            "vehicle_type" : type_vehicle,
            "plate_number" : plate_info,
            "camera_id"    : camera_id,
            "farm_id"      : self.farm_id_param
        }

        try:
            print('send')
            response_vehicle = requests.post(vehicle_api_url, json=api_message, headers=headers, timeout=30)
            pass
        except Exception as e:
            self.get_logger().info(f'error is {e}')
        if response_vehicle.status_code == 201:
            self.get_logger().info('Data successfully sent to camera api')
        elif response_vehicle.status_code == 500:
            self.get_refresh_token(response_vehicle)  # Use response_camera here
        else:
            '''If the request failed, log the error'''
            self.get_logger().error('Error posting data to API: {}'.format(response_vehicle.text))

    def get_refresh_token(self, original_response):
        refresh_url = 'http://102.37.220.214:8080/auth/refresh'
        headers = {'Authorization': f'Bearer {self.refresh_token}'}
        max_retries = 3
        retries = 0
        backoff_time = 2  # Time in seconds

        while retries < max_retries:
            try:
                refresh_response = requests.post(refresh_url, headers=headers)

                if refresh_response.status_code == 200:
                    new_access_token = refresh_response.json().get('access_token')

                    if new_access_token:
                        self.access_token = new_access_token
                        self.vehicle_ctrl(original_response, refresh_response)
                        return True  # Successfully refreshed token
                    else:
                        self.get_logger().error('Access token is empty or None.')

                else:
                    self.get_logger().error(f'Error refreshing Access token: {refresh_response.text}')

            except Exception as e:
                self.get_logger().info(f'Error in get_refresh_token: {e}')

            retries += 1
            self.get_logger().info(f'Retrying... ({retries}/{max_retries})')
            time.sleep(backoff_time)
            backoff_time *= 2  # Exponential backoff

        self.get_logger().error('Max retries reached. Could not refresh the token.')
        return False  # Failed to refresh token


def main(args=None):
    rclpy.init(args=args)
    camera = CameraControlNode()
    rclpy.spin(camera)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
