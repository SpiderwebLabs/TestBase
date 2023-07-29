import requests
import rclpy
import datetime
from rclpy.node import Node
from more_interfaces.msg import Trials1
from more_interfaces.srv import Mission
import sys
from rclpy.executors import ExternalShutdownException
import math
import json

drone_id_default = "b8a17037a16542fdac7599d4dd265322" 
farm_id_default = "6245bfdb760d4b829ccdfe6bee357a36" 
username_default = "NMUFieldB"
password_default = "12345"

# drone_id_default = "3e0af104f1874a68b0898fd7c65a9913" # Zurich drone use login Zurich
# farm_id_default = "d37e3e355af745c585bdf2b931be70b9" #Zurich farm
# username_default = "TestFlight"
# password_default = "12345"

# drone_id_default = "ad70c5c4e8484d18af6d9601eb023bf8" # Zurich drone use login Zurich
# farm_id_default = "7d3ab54f0c584bf3a0f8fdffe8e3c151" #Zurich farm
# username_default = "Zurich"
# password_default = "12345"

# drone_id_default = "c6348d8c6310401181da8496369f30fe" # Zurich drone use login Zurich
# farm_id_default = "5b15317d233c462d8b5c8863b1da26f1"#Zurich farm
# username_default = "NMU Flight"
# password_default = "12345"

class MyNode(Node):

    def __init__(self):
        '''Create a ROS2 subscriber node, initialize the node name'''
        super().__init__('mission_update')
        self.get_logger().info("API update node has been started")
        self.access_token = None
        self.refresh_token = None
        self.declare_parameter("drone_id", drone_id_default)
        self.declare_parameter("farm_id", farm_id_default)
        self.declare_parameter("username", username_default)
        self.declare_parameter("password", password_default)
        self.response_api = None
        self.response = None
        self.trigger = None

        self.drone_id_param = self.get_parameter("drone_id").get_parameter_value().string_value
        self.farm_id_param = self.get_parameter("farm_id").get_parameter_value().string_value
        self.username_param = self.get_parameter("username").get_parameter_value().string_value
        self.password_param = self.get_parameter("password").get_parameter_value().string_value
        '''Make a login request to get the JWT token'''

        login_url = 'http://102.37.220.214:8080/auth/login'
        login_data = {'username':  self.username_param, 'password': self.password_param}
        self.response = requests.post(login_url, json = login_data)

        if self.response.status_code == 200:
            '''If the login was successful, save the access token'''
            self.access_token = self.response.json()['access_token']
            self.refresh_token = self.response.json()['refresh_token']

        else:
            '''If the login failed, log the error'''
            self.get_logger().error('Error logging in: {}'.format(self.response.text))

        '''Set up a subscriber to listen for messages on the "data" topic'''
        
        self.mission_server = self.create_service(Mission, "mission_status", self.mission_callback)
        self.drone_state_server = self.create_service(Mission, "station_state", self.station_callback)

    def mission_callback(self, request, response):
        '''callback function for mission server'''
        mission_api_url = f"http://102.37.220.214:8080/missions/{self.farm_id_param}/update_mission_status"
        headers = {'Authorization': 'Bearer {}'.format(self.access_token)}
        mission_status = int(request.mission_status)
        self.get_logger().info(f"mission_status:{mission_status}")
        self.mission_id = request.mission_id
        self.mission_message = {'mission_status': mission_status,
                              "mission_id":self.mission_id}
        
        '''send a Post to the API, authentication with the JWT token'''
        try:
            self.response_mission = requests.put(mission_api_url, json = self.mission_message, headers=headers, timeout=30)
        except Exception as e:
            self.get_logger().info(f'error is {e}')
        if self.response_mission.status_code == 200:
            '''If the request was successful, log the response'''
            self.get_logger().info('Data successfully posted  Mission Status to API')
        elif self.response_mission.status_code == 500:
            self.get_logger().info("Getting the refresh token")
            '''If the request failed, refresh the token and try again'''
            self.trigger = True
            self.get_refresh_token(request, response, self.trigger)
            
        else:
            '''If the request failed, log the error'''
            self.get_logger().error('Error posting data to API: {}'.format(self.response_mission.text))
        response.success =  "done"
        return response
    
    def station_callback(self, request, response):
        '''callback function for mission server'''
        station_api_url = f"http://102.37.220.214:8080/drone/{self.drone_id_param}/update_base_station_status"
        headers = {'Authorization': 'Bearer {}'.format(self.access_token)}
        station_status = int(request.mission_status)
        self.get_logger().info(f"station_status:{station_status}")
        #self.mission_id = request.mission_id
        self.station_message = {'base_station_status': station_status}
        
        '''send a Post to the API, authentication with the JWT token'''
        try:
            self.response_station = requests.post(station_api_url, json = self.station_message, headers=headers, timeout=30)
        except Exception as e:
            self.get_logger().info(f'error is {e}')
        if self.response_station.status_code == 200:
            '''If the request was successful, log the response'''
            self.get_logger().info('Data successfully posted  Station Status to API')
        elif self.response_station.status_code == 500:
            self.get_logger().info("Getting the refresh token")
            '''If the request failed, refresh the token and try again'''
            self.trigger = False
            self.get_refresh_token(request, response, self.trigger)
            
        else:
            '''If the request failed, log the error'''
            self.get_logger().error('Error posting data to API: {}'.format(self.response_station.text))
        response.success =  "done"
        return response

    def get_refresh_token(self, request, response, trig):
        '''Send a refresh request to the API to get a new Access token'''
        refresh_url = 'http://102.37.220.214:8080/auth/refresh'
        headers = {'Authorization': 'Bearer {}'.format(self.refresh_token)}
        
        max_retries = 3  # You can set the maximum number of retries here
        retries = 0

        while retries < max_retries:
            try:
                response = requests.post(refresh_url, headers=headers)
                if response.status_code == 200:
                    new_access_token = response.json()['access_token']
                    if new_access_token is not None and new_access_token != "":
                        self.access_token = new_access_token
                        if trig:
                            self.mission_callback(request, response)  # Call the mission_callback again with the updated access_token
                        else:
                            self.station_callback(request, response)
                        break
                    else:
                        self.get_logger().error('Access token is empty or None.')
                        retries += 1
                else:
                    self.get_logger().error('Error refreshing Access token: {}'.format(response.text))
                    retries += 1
            except Exception as e:
                self.get_logger().info(f'Error in get_refresh_token: {e}')
                retries += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.try_shutdown()
    node.destroy_node()


if __name__ == '__main__':
    main()
