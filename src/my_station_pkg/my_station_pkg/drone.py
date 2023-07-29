import asyncio
import threading
from threading import Lock
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from mavsdk.mission import (MissionItem, MissionPlan)
from more_interfaces.srv import Comms
from more_interfaces.msg import Trials1
from mavsdk import System
from geopy import distance
from decimal import Decimal


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Initialize the drone
        self.drone = System()
        self.telemetry = None

        # Create a publisher for telemetry data
        self.telemetry_pub = self.create_publisher(Trials1, 'status_update', 10)

        # Create a server to receive actions from the system controller node
        self.action_server = self.create_service(
            Comms, 'set_comms', self.handle_action)

        # Telemetry publisher timer
        self.timer_ = self.create_timer(0.5, self.callback_status_update)

        # Connect to the drone
        self.connect_to_drone()

        #Lock object to avoid thread race 
        self.lock = Lock()

       

        # Initialize the variables
        self.drone_action = "DoNothing"
        self.mission_complete = False
        self.flight_longitude = None
        self.flight_latitude = None
        self.flight_altitude = None
        self.desired_yaw = None
        self.land_delay = -1
        self.flight_mode = 0

        # Initialize the status info
        self.battery_percentage = 0.0
        self.gps_info = 0
        self.armed = False
        self.in_air = False
        self.landed_state = 0
        self.pos_latitude = 0.0
        self.pos_longitude = 0.0
        self.pos_altitude = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0
        self.current_yaw = 0.0
        self.mission_complete = False

    def callback_status_update(self):
        '''Function to get status information and publish'''
        msg = Trials1()
        # Updating drone status information
        msg.battery_percentage = self.battery_percentage
        msg.is_in_air = self.in_air
        msg.armed = self.armed
        msg.landed_state = self.landed_state
        msg.pos_latitude = self.pos_latitude
        msg.pos_longitude = self.pos_longitude
        msg.pos_altitude = self.pos_altitude
        msg.velocity_x = self.velocity_x
        msg.velocity_y = self.velocity_y
        msg.velocity_z = self.velocity_z
        msg.current_yaw = self.current_yaw
        msg.mission_done = self.mission_complete
        # Publishing drone status information
        self.get_logger().info("Am publishing the information")
        self.telemetry_pub.publish(msg)

    def connect_to_drone(self):
        self.get_logger().info('Waiting for the drone to connect...')

        loop = asyncio.get_event_loop()
        try:
            loop.run_until_complete(self._connect_to_drone())
        except Exception as e:
            self.get_logger().error(f"Drone connection failed: {str(e)}")

        self.get_logger().info('Drone connected')

    async def _connect_to_drone(self):
        await self.drone.connect(system_address="udp://:14540")

    async def handle_action(self, request, response):
        self.get_logger().info("set_drone_action")
        self.drone_action = request.message
        self.get_logger().info(self.drone_action)
        self.flight_longitude = request.pos_longitude
        self.flight_latitude = request.pos_latitude
        self.flight_altitude = request.pos_altitude
        self.desired_yaw = request.turn_yaw

        # Acquire the lock
        self.lock.acquire()

        try:

            if self.drone_action == 'Clearmission':
                await self.perform_clearmission()
                response.connect_success = True

            elif self.drone_action == 'Takeoff':
                await self.perform_takeoff()
                response.connect_success = True
            
                

            elif self.drone_action == "Land":
                await self.perform_landing()
                self.drone_action = None
                response.connect_success = True

            elif self.drone_action == "Goto":
                await self.perform_mission(
                    Decimal(self.flight_latitude),
                    Decimal(self.flight_longitude),
                    Decimal(self.flight_altitude)
                )
                response.connect_success = True
            elif self.drone_action == "Goback":
                await self.perform_returnhome()
                response.connect_success = True

            elif self.drone_action == "Turn":
                await self.perform_turn(
                    Decimal(self.flight_latitude),
                    Decimal(self.flight_longitude),
                    Decimal(self.flight_altitude),
                    self.desired_yaw
                )
                response.connect_success = True
            else:
                self.get_logger().warn("Unkwown action request")
        
        finally:
            # Release the lock
            self.lock.release()
       
        return response

        
    async def perform_takeoff(self):
        await self.drone.action.arm()
        await self.drone.action.takeoff()
       

    async def perform_clearmission(self):
        await self.drone.action.hold()

    async def perform_landing(self):
        await self.drone.action.land()

    async def perform_mission(self, lat, lon, alt):
        await self.drone.action.arm()
        mission_items = []
        mission_items.append(MissionItem(
            lat,
            lon,
            alt,
            3.0,
            True,
            float('nan'),
            float('nan'),
            MissionItem.CameraAction.NONE,
            5.0,
            float('nan'),
            3.0,
            float('nan'),
            float('nan')
        ))
        mission_plan = MissionPlan(mission_items)
        await self.drone.mission.upload_mission(mission_plan)
        await self.drone.mission.start_mission()

    async def perform_returnhome(self):
        await self.drone.action.return_to_launch()

    async def perform_turn(self, lat, lon, alt, yaw):
        await self.drone.action.goto_location(lat, lon, alt, yaw)

    def start_telemetry_thread(self):
        loop = asyncio.get_event_loop()
        telemetry_thread = threading.Thread(
            target=self.publish_telemetry, args=(loop,))
        telemetry_thread.start()

    def publish_telemetry(self, loop):
        asyncio.set_event_loop(loop)

        if self.telemetry is None:
            self.telemetry = self.drone.telemetry

        #rate = self.create_rate(1.0)  # 1 Hz rate

        
            

        async def retrieve_telemetry_data():
            
            while rclpy.ok():
                # Acquire the lock
                self.lock.acquire()

                try:

                    self.mission_complete_check()

                    async for battery in self.telemetry.battery():
                        self.get_logger().info(f"Battery: {battery.remaining_percent}")
                        self.battery_percentage = battery.remaining_percent
                        break

                    async for gps_info in self.drone.telemetry.gps_info():
                        self.get_logger().info(f"GPS info: {gps_info}")
                        self.gps_info = gps_info
                        break

                    async for in_air in self.drone.telemetry.in_air():
                        self.get_logger().info(f"in_air: {in_air}")
                        self.in_air = in_air
                        break

                    async for armed in self.drone.telemetry.armed():
                        self.get_logger().info(f"armed: {armed}")
                        self.armed = armed
                        break

                    async for land_detected in self.drone.telemetry.landed_state():
                        self.landed_status = self.landing_state_check(land_detected)
                        break

                    async for position in self.drone.telemetry.position():
                        self.get_logger().info(f"position_lat: {position.latitude_deg}")
                        self.get_logger().info(f"position_lon: {position.longitude_deg}")
                        self.get_logger().info(f"position_alt: {position.relative_altitude_m}")
                        self.get_logger().info(f"position_alt: {position.absolute_altitude_m}")
                        self.pos_latitude = position.latitude_deg
                        self.pos_longitude = position.longitude_deg
                        self.pos_altitude = position.relative_altitude_m
                        break

                    async for velocity in self.drone.telemetry.velocity_ned():
                        self.velocity_x = velocity.north_m_s
                        self.get_logger().info(f"velocity_x: {self.velocity_x}")
                        self.velocity_y = velocity.east_m_s
                        self.get_logger().info(f"velocity_y: {self.velocity_y}")
                        self.velocity_z = velocity.down_m_s
                        self.get_logger().info(f"velocity_z: {self.velocity_z}")
                        break

                    async for yaw_info in self.drone.telemetry.attitude_euler():
                        self.get_logger().info(f"yaw_angle: {yaw_info.yaw_deg}")
                        self.current_yaw = yaw_info.yaw_deg
                        break

                    # async for flight_info in self.drone.telemetry.flight_mode():
                    #     self.get_logger().info(f"flight_mode: {flight_info}")

                    self.get_logger().info(
                        f"mission_finished: {await self.drone.mission.is_mission_finished()}")
                
                finally:
                    # Release the lock
                    self.lock.release()

                await asyncio.sleep(2)  # Delay for 1 second
            

        loop.run_until_complete(retrieve_telemetry_data())
        loop.close()

    def mission_complete_check(self):
        '''Function to check if drone has reached the target location'''
        cur_pos = (self.pos_latitude, self.pos_longitude)
        target_pos = (self.flight_latitude, self.flight_longitude)
        self.mission_complete = distance.distance(cur_pos, target_pos).m < 2.0
        self.get_logger().info(f"distance_mission_complete: {self.mission_complete}")

    def landing_state_check(self, lander_state):
        '''function to check drone landing status and assign an integer to each state'''
        match lander_state:
            case lander_state.ON_GROUND:
                return 0
            case lander_state.IN_AIR:
                return 2
            case lander_state.TAKING_OFF:
                return 1
            case lander_state.LANDING:
                return 3
            case lander_state.UNKNOWN:
                return 4

    def run(self):
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        executor.spin()

    def shutdown(self):
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    drone_controller.start_telemetry_thread()
    drone_controller.run()
    drone_controller.shutdown()


if __name__ == '__main__':
    main()
