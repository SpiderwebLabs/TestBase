import requests
import rclpy
import datetime
from rclpy.node import Node
from more_interfaces.msg import Trials1
import math
from std_msgs.msg import Bool
from more_interfaces.srv import Trigger
import threading
import json
import rclpy
import sys
from rclpy.executors import ExternalShutdownException


# drone_id_default = "3e0af104f1874a68b0898fd7c65a9913" # TestFlight drone use login Zurich
# farm_id_default = "d37e3e355af745c585bdf2b931be70b9" #TestFlight farm
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

drone_id_default = "b8a17037a16542fdac7599d4dd265322" 
farm_id_default = "6245bfdb760d4b829ccdfe6bee357a36" 
username_default = "NMUFieldB"
password_default = "12345"


class MyNode(Node):

    def __init__(self):
        '''Create a ROS2 subscriber node, initialize the node name'''
        super().__init__('api_update_node')
        self.get_logger().info("API update node has been started")
        self.access_token = None
        self.refresh_token = None
        self.declare_parameter("drone_id", drone_id_default)
        self.declare_parameter("farm_id", farm_id_default)
        self.declare_parameter("username", username_default)
        self.declare_parameter("password", password_default)
        self.response = None
        self.started = False
        self.subscription = None
        self.thread = None
        self.service = self.create_service(Trigger, 'start_stop', self.start_stop_callback)
        self.drone_id_param = self.get_parameter("drone_id").get_parameter_value().string_value
        self.farm_id_param = self.get_parameter("farm_id").get_parameter_value().string_value
        self.username_param = self.get_parameter("username").get_parameter_value().string_value
        self.password_param = self.get_parameter("password").get_parameter_value().string_value
        

        '''Make a login request to get the JWT token'''

        login_url = 'http://102.37.220.214:8080/auth/login'
        login_data = {'username':  self.username_param, 'password': self.password_param}
        response = requests.post(login_url, json = login_data)
        if response.status_code == 200:
            '''If the login was successful, save the access token'''
            self.access_token = response.json()['access_token']
            self.refresh_token = response.json()['refresh_token']
            

        else:
            '''If the login failed, log the error'''
            self.get_logger().error('Error logging in: {}'.format(response.text))

        

    def start_func(self):
        self.started = True
        if self.started:
            '''Set up a subscriber to listen for messages on the "data" topic'''
        self.subscription = self.create_subscription(
            Trials1,
            "status_update", self.data_callback, 10)
        
         

    def stop_func(self):
        # Stop the functionality
        self.started = False
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None
        # Check if subscription has already been destroyed
        if self.subscription is None:
            return
        # Enter loop to handle remaining callbacks
        while rclpy.ok():
            rclpy.spin_once(self)

            

    def start_stop_callback(self, request, response):
        self.started = request.trigger
        if self.started:
            self.thread = threading.Thread(target=self.start_func)
            self.thread.start()
            response.success = True
            return response

        else:
            self.stop_func()
            response.success = True
            return response
    
    def data_callback(self, msg):
        '''This function is called when a message is received on the "data" topic'''
        api_url = f'http://102.37.220.214:8080/drone/{self.drone_id_param}/status'

        headers = {'Authorization': 'Bearer {}'.format(self.access_token)}
        self.is_in_air = msg.is_in_air
        self.armed = msg.armed
        self.battery = msg.battery_percentage
        self.y_coordinate = msg.pos_latitude
        self.x_coordinate = msg.pos_longitude
        self.z_coordinate = msg.pos_altitude
        self.lander_state = msg.landed_state
        vel_x = msg.velocity_x
        vel_y = msg.velocity_y
        vel_z = msg.velocity_z

        self.speed = math.sqrt((vel_x)**2+(vel_y)**2+(vel_z)**2)
        self.get_logger().info(f"{self.battery}")
        self.get_logger().info(f"{self.armed}")
        self.get_logger().info(f"{self.is_in_air}")
        self.get_logger().info(f"{self.speed}")
        self.get_logger().info(f"msg:{self.lander_state}")
        self.message = {'is_in_air': self.is_in_air,
                        'armed': self.armed,
                        'battery': self.battery,
                        'x_coordinate': self.x_coordinate,
                        'y_coordinate': self.y_coordinate,
                        'altitude': self.z_coordinate,
                        'landing_status': self.lander_state,
                        'speed': self.speed,
                        "time_last_updated": str(datetime.datetime.now())}
        

        '''Send a POST request to the API, authenticated with the JWT token'''
        try:
            self.response_api = requests.post(api_url, json=self.message, headers=headers, timeout = 30)
        except Exception as e:
            self.get_logger().info(f'error is {e}')

        if self.response_api.status_code == 200:
            '''If the request was successful, log the response'''
            self.get_logger().info('Data successfully posted to API')
        elif self.response_api.status_code == 500:
            '''If the request failed, refresh the token and try again'''
            self.get_refresh_token(msg)
        else:
            '''If the request failed, log the error'''
            self.get_logger().error('Error posting data to API: {}'.format(self.response_api.text))


    def get_refresh_token(self, msg):
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
                        self.data_callback(msg)  # Call the api_callback again with the updated access_token
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
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
