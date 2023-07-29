import rclpy
from rclpy.node import Node
from more_interfaces.msg import Trials1
from more_interfaces.srv import Trigger
from mavsdk import System
import asyncio

class TelemetryPublisherNode(Node):
    def __init__(self):
        super().__init__('telemetrypoll_node')
        self.drone = System()

        self.publisher_ = self.create_publisher(Trials1, 'status_update', 10)

        # Define the service to start and stop telemetry polling
        #self.srv = self.create_service(Trigger, 'start_stop_telemetry', self.start_stop_telemetry_callback)
        self.is_polling = True
        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self.connect_to_drone())
        self.loop.run_until_complete(self.telemetry_polling())
        #self.loop.create_task(self.telemetry_polling())
        self.create_timer(0.5,self.telemetry_polling)
        # self.create_timer(0.7, self.telem_pub_callback)
        self.mission_complete = False
        self.flight_latitude = None
        self.flight_longitude = None
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitudeSealevel = None
        self.flight_mode = 3
        
        self.armed = False
        self.in_air = False
        self.gps = 0
        self.landed_status = 0
        self.pos_altitude = 0.0
        self.altitudeSealevel = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0
        self.yaw = 0.0

    async def connect_to_drone(self):
        self.get_logger().info('Waiting for the drone to connect...')

        try:
            await self._connect_to_drone()
        except Exception as e:
            self.get_logger().error(f"Drone connection failed: {str(e)}")

        self.get_logger().info('Drone connected')

    async def _connect_to_drone(self):
        #await self.drone.connect(system_address="serial:///dev/ttyRadiotelem:57600")
        #await self.drone.connect(system_address="udp://:14540")
        #await self.drone.connect(system_address = "serial:///dev/ttyUSB0:57600")
        await self.drone.connect(system_address ="serial:///dev/ttyUSB1:57600")
        #await self.drone.core.set_mavlink_timeout(2.0)
        
    def getcoordinates_callback(self, request, response):
        self.flight_latitude = request.pos_latitude
        self.flight_longitude = request.pos_longitude
        self.get_logger().info('Cordinatesupdated')
        response.connect_success = True
        return response
    
    # async def telem_pub_callback(self):
    #     msg = Trials1()
    #     msg.armed = self.armed
    #     msg.is_in_air = self.in_air
    #     msg.gps_satellites = self.gps
    #     #msg.battery_percentage = battery.remaining_percent
    #     msg.flight_mode = self.flight_mode
    #     msg.landed_state = self.landed_status
    #     msg.pos_altitude = self.pos_altitude 
    #     msg.rel_altitude = self.altitudeSealevel
    #     msg.pos_longitude = self.longitude
    #     msg.pos_latitude = self.latitude
    #     msg.velocity_x = self.velocity_x
    #     msg.velocity_y = self.velocity_y
    #     msg.velocity_z = self.velocity_z
    #     msg.mission_done = self.mission_complete
    #     msg.current_yaw = self.yaw
    #     self.publisher_.publish(msg)


    async def telemetry_polling(self):
        print('I am here')
        if self.is_polling == True:
            while rclpy.ok():
            #while True:
                # try:
                #     async for battery in self.drone.telemetry.battery():  # get drone battery status
                #         self.get_logger().info(f"battery_state: {battery.remaining_percent}")
                #         break
                # except Exception as e:
                #     self.get_logger().error(f"Error in battery telemetry: {e}")

                try:
                    async for gps_info in self.drone.telemetry.gps_info():
                        self.get_logger().info(f"GPS info: {gps_info}")
                        self.gps = gps_info.num_satellites
                        break
                except Exception as e:
                    self.get_logger().error(f"Error in GPS telemetry: {e}")

                try:
                    async for in_air in self.drone.telemetry.in_air():  # get drone flying status
                        self.get_logger().info(f"in_air:{in_air}")
                        self.in_air = in_air
                        break
                except Exception as e:
                    self.get_logger().error(f"Error in in-air telemetry: {e}")

                try:
                    async for armed in self.drone.telemetry.armed():  # get drone arming status
                        self.get_logger().info(f"armed:{armed}")
                        self.armed = armed
                        break
                except Exception as e:
                    self.get_logger().error(f"Error in armed telemetry: {e}")

                try:
                    async for land_detected in self.drone.telemetry.landed_state():  # get landing status
                        self.landed_status = self.landing_state_check(land_detected)
                        self.get_logger().info(f"landed_state:{self.landed_status}")
                        break
                except Exception as e:
                    self.get_logger().error(f"Error in landed state telemetry: {e}")

                try:
                    async for position in self.drone.telemetry.position():  # get drone position
                        self.get_logger().info(f"position_lat:{position.latitude_deg}")
                        self.get_logger().info(f"position_lon:{position.longitude_deg}")
                        self.get_logger().info(f"position_alt:{position.relative_altitude_m}")
                        self.get_logger().info(f"position_alt:{position.absolute_altitude_m}")
                        self.longitude = position.longitude_deg
                        self.latitude = position.latitude_deg
                        self.pos_altitude = position.relative_altitude_m
                        self.altitudeSealevel = position.absolute_altitude_m
                        break
                except Exception as e:
                    self.get_logger().error(f"Error in position telemetry: {e}")

                try:
                    async for velocity in self.drone.telemetry.velocity_ned():
                        self.velocity_x = velocity.north_m_s
                        self.get_logger().info(f"velocity_x:{self.velocity_x}")
                        self.velocity_y = velocity.east_m_s
                        self.get_logger().info(f"velocity_y: {self.velocity_y}")
                        self.velocity_z = velocity.down_m_s
                        self.get_logger().info(f"velocity_z:{self.velocity_z}")
                        break
                except Exception as e:
                    self.get_logger().error(f"Error in velocity telemetry: {e}")

                try:
                    async for yaw_info in self.drone.telemetry.attitude_euler():
                        self.get_logger().info(f"yaw_angle: {yaw_info.yaw_deg}")
                        self.yaw = yaw_info.yaw_deg
                        break
                except Exception as e:
                    self.get_logger().error(f"Error in attitude telemetry: {e}")

                try:
                    async for flight_info in self.drone.telemetry.flight_mode():
                        self.get_logger().info(f"flight_mode: {flight_info}")
                        self.flight_mode = self.mission_complete_function(flight_info)
                        break
                except Exception as e:
                    self.get_logger().error(f"Error in flight mode telemetry: {e}")

                msg = Trials1()
                msg.armed = self.armed
                msg.is_in_air = self.in_air
                msg.gps_satellites = self.gps
                #msg.battery_percentage = battery.remaining_percent
                msg.flight_mode = self.flight_mode
                msg.landed_state = self.landed_status
                msg.pos_altitude = self.pos_altitude 
                msg.rel_altitude = self.altitudeSealevel
                msg.pos_longitude = self.longitude
                msg.pos_latitude = self.latitude
                msg.velocity_x = self.velocity_x
                msg.velocity_y = self.velocity_y
                msg.velocity_z = self.velocity_z
                #msg.mission_done = self.mission_complete
                msg.current_yaw = self.yaw
                self.publisher_.publish(msg)

            

    def landing_state_check(self, lander_state):
        landing_states = {
            lander_state.ON_GROUND: 0,
            lander_state.IN_AIR: 2,
            lander_state.TAKING_OFF: 1,
            lander_state.LANDING: 3,
            lander_state.UNKNOWN: 4
        }

        return landing_states.get(lander_state, 4)  # Default to 4 if the key is not found


    def mission_complete_function(self, flight_info):
        flight_modes = {
            flight_info.HOLD: 3,
            flight_info.MISSION: 4,
            flight_info.TAKEOFF: 2,
            flight_info.LAND: 6,
            flight_info.RETURN_TO_LAUNCH: 5,
            flight_info.ACRO: 8
        }

        self.flight_mode = flight_modes.get(flight_info, 9)  # Default to 9 if the key is not found
        return self.flight_mode


            



    # async def start_stop_telemetry_callback(self, request, response):
    #     if request.trigger == True:
    #         # Start telemetry polling
    #         self.get_logger().info("Telemetry polling started")
    #         self.is_polling = True
    #         self.get_logger().info('Polling in progress...')
    #         response.success = True
    #     elif request.trigger == False:
    #         # Stop telemetry polling if it is running
    #         self.is_polling = False
    #         self.get_logger().info("Telemetry polling stopped")
    #         response.success = True
    #     return response
    


def main(args=None):
    rclpy.init(args=args)
    telemetry_publisher = TelemetryPublisherNode()
    rclpy.spin(telemetry_publisher)
    telemetry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
