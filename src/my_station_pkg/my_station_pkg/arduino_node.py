import rclpy
from rclpy.node import Node
import serial
from more_interfaces.srv import Arduino1
from std_msgs.msg import String
from more_interfaces.srv import Mission
import sys
from rclpy.executors import ExternalShutdownException



class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_node')
        #load_dotenv()
        #self.value=os.getenv('arduino_connect','/dev/ttyACM0')
        #print(self.value)
        #self.value = "/dev/ttyUSB1"
        self.value = "/dev/ttyArduino"
        #initialize serial communication with Arduino
        self.ser = serial.Serial(self.value, 9600)
        self.ser.flushInput()

        #create a publisher for the "data" topic
        self.station_status_pub = self.create_publisher(String, 'arduino_action_complete', 10)
        timer_period = 0.5  # seconds
        self.timer_ = self.create_timer(timer_period, self.publish_data)
        # create a service for sending messages to the Arduino
        self.srv = self.create_service(
            Arduino1, 'send_message', self.handle_send_message)
        self.drone_state_client = self.create_client(Mission, "station_state")
        self.mission_client = self.create_client(Mission, "mission_status")
        self.get_logger().info('Arduino service started')
        self.ser_flag = False  # flag to allow the publisher to access the serial port
        self.mission_id = " "
        self.trigger = False
        self.send = None
        

    def handle_send_message(self, request, response):
        '''Function to send commands to arduino and to api_update node when a request is recieved'''
        self.trigger = request.trigger
        self.mission_id = request.mission_id
        self.get_logger().info(f"{self.mission_id}")
        self.req = Mission.Request()
        print(request.message)
        if not self.trigger:
            if request.message == "G":
                self.req.mission_status = 1.0 #charging on
            elif request.message == "H":
                self.req.mission_status = 2.0 #open hatch
            elif request.message == "U":
                self.req.mission_status = 5.0 #uncentering
            elif request.message == "S":
                self.req.mission_status = 0.0 #Charging off
            elif request.message == "T":
                self.req.mission_status = 3.0 #close hatch
            elif request.message == "C":
                self.req.mission_status = 4.0# centering
            elif request.message == "R":
                self.req.mission_status = 6.0 #platform up
            elif request.message == "D":
                self.req.mission_status = 7.0#platformdown
            self.drone_state_client.call_async(self.req)

        else:
            self.mission_id = request.mission_id
            
        self.ser_flag = True  # flag to stop the publisher from accessing the serial port
        # send the message to the Arduino
        self.ser.write(request.message.encode())
        self.ser_flag = False  # flag to allow the publisher to access the serial port
        response.outcome = True  # set the response data for the service
        return response

    def publish_data(self):
        '''Function to publish the state of the Drone Station'''
        if self.ser.in_waiting > 0 and self.ser_flag == False:
            msg = String()
            # read the response from the Arduino and publish it on data topic
            msg.data = self.ser.readline().decode().strip()
            print(msg.data)
            if self.trigger:
                '''This part sends message to api during system initilization process'''
                self.send = msg.data
                self.request =  Mission.Request()
                self.req = Mission.Request()
                if self.send == "Hatchopened": 
                    self.req.mission_status = 2.0
                    self.drone_state_client.call_async(self.req)
                    self.request.mission_status = 11.0 # sent to signal to say initialization process done
                    self.request.mission_id = self.mission_id
                    self.mission_client.call_async(self.request)
                    self.mission_id = " "
                    self.send = None

                elif self.send == "Uncentred":
                    self.req.mission_status = 5.0
                    self.drone_state_client.call_async(self.req)
                    self.request.mission_status = 11.0 # sent to signal to say initialization process done
                    self.request.mission_id = self.mission_id
                    self.mission_client.call_async(self.request)
                    self.mission_id = " "
                    self.send = None

                elif self.send == "Hatchclosed":
                    self.req.mission_status = 3.0
                    self.drone_state_client.call_async(self.req)
                    self.request.mission_status = 11.0 # sent to signal to say initialization process done
                    self.request.mission_id = self.mission_id
                    self.mission_client.call_async(self.request)
                    self.mission_id = " "
                    self.send = None

                elif self.send == "Centred":
                    self.req.mission_status = 4.0
                    self.drone_state_client.call_async(self.req)
                    self.request.mission_status = 11.0 # sent to signal to say initialization process done
                    self.request.mission_id = self.mission_id
                    self.mission_client.call_async(self.request)
                    self.mission_id = " " 
                    self.send = None

                elif self.send == "Platformup":
                    self.req.mission_status = 6.0
                    self.drone_state_client.call_async(self.req)
                    self.request.mission_status = 11.0 # sent to signal to say initialization process done
                    self.request.mission_id = self.mission_id
                    self.mission_client.call_async(self.request)
                    self.mission_id = " " 
                    self.send = None

                elif self.send == "Platformdown":
                    self.req.mission_status = 7.0
                    self.drone_state_client.call_async(self.req)
                    self.request.mission_status = 11.0 # sent to signal to say initialization process done
                    self.request.mission_id = self.mission_id
                    self.mission_client.call_async(self.request)
                    self.mission_id = " " 
                    self.send = None
            self.station_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    arduino_node = ArduinoNode()
    rclpy.spin(arduino_node)
    arduino_node.destroy_node()
    


if __name__ == '__main__':
    main()
