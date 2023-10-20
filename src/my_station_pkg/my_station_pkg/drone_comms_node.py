import rclpy
from rclpy.node import Node
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from more_interfaces.srv import Comms, Trigger
from more_interfaces.msg import Trials1
from decimal import Decimal
from mavsdk.offboard import Attitude, AttitudeRate
import asyncio
import time

class ControlServerNode(Node):
    def __init__(self):
        super().__init__('drone_comms_node')
        self.drone = System()


        self.arm_service = self.create_service(Comms, 'arm', self.arm_callback)
        self.takeoff_service = self.create_service(Comms, 'takeoff', self.takeoff_callback)
        self.goto_service = self.create_service(Comms, 'goto', self.mission_callback)
        self.land_service = self.create_service(Comms, 'land', self.land_callback)
        self.turn_service = self.create_service(Comms, 'turn', self.turn_callback)
        self.return_service = self.create_service(Comms, 'returnback', self.returnback_callback)
        self.missionclear_service = self.create_service(Comms, 'clear_mission', self.clearmission_callback)
        self.pausemission_service = self.create_service(Comms, 'pause_mission', self.pausemission_callback)
        self.reroute_service = self.create_service(Comms, 'reroute', self.reroute_callback)
        self.telemsub = self.create_subscription(Trials1, 'status_update', self.telemcallback, 10)
        self.telempause = self.create_client(Trigger, 'start_stop_telemetry')
        


        #start an event loop
        self.loop = asyncio.get_event_loop()
        self.start_pause = Trigger.Request()

        # Connect to the drone
        self.loop.run_until_complete(self.connect_to_drone())

        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.altabove = 0.0
    async def telemcallback(self, msg):
        self.altabove = msg.rel_altitude
    async def connect_to_drone(self):
        self.get_logger().info('Waiting for the drone to connect...')

        try:
            await self._connect_to_drone()
        except Exception as e:
            self.get_logger().error(f"Drone connection failed: {str(e)}")

        self.get_logger().info('Drone connected')

    async def _connect_to_drone(self):
        #await self.drone.connect(system_address="serial:///dev/ttyRadiotelem:57600")
        await self.drone.connect(system_address="udp://:14540")
        #await self.drone.connect(system_address ="serial:///dev/ttyUSB0:57600")
        await self.drone.core.set_mavlink_timeout(1.0)


    async def pausemission_callback(self, request, response):
        print("pause")
        self.loop.run_until_complete(self.perform_pausemission())
        response.connect_success
        return response
    
    async def perform_pausemission(self):
        await self.drone.mission.pause_mission()

    async def reroute_callback(self, request, response):
        print("reroute")
        self.start_pause.trigger = False
        self.telempause.call_async(self.start_pause)
        self.loop.run_until_complete(self.perform_reroute(Decimal(request.pos_altitude), Decimal(request.pos_longitude), Decimal(request.pos_latitude)))
        response.connect_success
        return response
    
    async def perform_reroute(self, alt, lon, lat):
        self.latitude = lat
        self.longitude = lon
        self.altitude = alt
        #await self.drone.action.goto_location(lat, lon, self.altabove + 3.0 , 0.0)
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
                                        float('nan')))
        mission_plan = MissionPlan(mission_items)
        while True:
            try:
                await self.drone.mission.upload_mission(mission_plan)
                
            except Exception as e:
                self.get_logger().error(f"Drone connection failed: {str(e)}")
                await asyncio.sleep(2)
            else:
                try:
                    await self.drone.mission.start_mission()
                except Exception as e:
                    self.get_logger().error(f"Drone connection failed: {str(e)}")
                    await asyncio.sleep(2)
                else:
                    self.get_logger().info(f'Drone going to point, altitude: {alt}, longitude: {lon}, latitude: {lat}')
                    self.start_pause.trigger = True
                    self.telempause.call_async(self.start_pause)
                    break
    
        
    


    async def clearmission_callback(self, request, response):
        print("clear")
        self.loop.run_until_complete(self.perform_missionclear())
        response.connect_success = True
        return response
    
    async def perform_missionclear(self):
        #await self.drone.mission.clear_mission()
        pass

    
    async def returnback_callback(self, request, response):
        self.loop.run_until_complete(self.perform_return())
        response.connect_success = True
        return response
    
    async def perform_return(self):
        await self.drone.action.return_to_launch()
        self.get_logger().info("Drone returning to launch")


    async def arm_callback(self, request, response):
        self.loop.run_until_complete(self.perform_arm())
        response.connect_success = True
        return response
    
    async def perform_arm(self):
        await self.drone.action.arm()
        self.get_logger().info('Drone armed')


    async def takeoff_callback(self, request, response): 
        self.loop.run_until_complete(self.perform_takeoff())
        response.connect_success = True
        return response
    
    async def perform_takeoff(self):
        await self.drone.action.takeoff()
        self.get_logger().info('Drone taking off')

    async def mission_callback(self, request, response):
        print('mission send')
        self.start_pause.trigger = False
        self.telempause.call_async(self.start_pause)
        if request.trigger:
            self.loop.run_until_complete(self.avoid_obstacles(request))
        else:
            self.loop.run_until_complete(self.perform_mission(Decimal(request.pos_altitude), Decimal(request.pos_longitude), Decimal(request.pos_latitude)))
        response.connect_success = True
        return response
    
    async def avoid_obstacles(self, request):
        # Given list of coordinates
        coordinates = request.avoid_obstacle_waypoints

        mission_items = []

        # Iterate through the coordinates two at a time
        for i in range(0, len(coordinates), 2):
            longitude = float(coordinates[i])
            latitude = float(coordinates[i + 1])

            mission_item = MissionItem(
                latitude,
                longitude,
                3.0,
                3.0,
                True,
                float('nan'),
                float('nan'),
                MissionItem.CameraAction.NONE,
                0.0,
                float('nan'),
                3.0,
                float('nan'),
                float('nan')
            )

            mission_items.append(mission_item)

        # Create a MissionPlan from the mission_items
        mission_plan = MissionPlan(mission_items)
        try:
            await self.drone.mission.upload_mission(mission_plan)
            
        except Exception as e:
            self.get_logger().error(f"Drone connection failed: {str(e)}")
        else:
            try:
                await self.drone.action.arm()
            except Exception as e:
                self.get_logger().error(f"Drone connection failed: {str(e)}")
                await asyncio.sleep(2)
            else:
                try:
                    await self.drone.mission.start_mission()
                except Exception as e:
                    self.get_logger().error(f"Drone connection failed: {str(e)}")
                    await asyncio.sleep(2)
                else:
                    self.get_logger().info(f'Drone going to point')
                    self.start_pause.trigger = True
                    self.telempause.call_async(self.start_pause)




    
    async def perform_mission(self, alt, lon, lat):
        self.latitude = lat
        self.longitude = lon
        self.altitude = alt
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
                                        float('nan')))
        mission_plan = MissionPlan(mission_items)
        
        try:
            await self.drone.mission.upload_mission(mission_plan)
            
        except Exception as e:
            self.get_logger().error(f"Drone connection failed: {str(e)}")
        else:
            try:
                await self.drone.action.arm()
            except Exception as e:
                self.get_logger().error(f"Drone connection failed: {str(e)}")
                await asyncio.sleep(2)
            else:
                try:
                    await self.drone.mission.start_mission()
                except Exception as e:
                    self.get_logger().error(f"Drone connection failed: {str(e)}")
                    await asyncio.sleep(2)
                else:
                    self.get_logger().info(f'Drone going to point, altitude: {alt}, longitude: {lon}, latitude: {lat}')
                    self.start_pause.trigger = True
                    self.telempause.call_async(self.start_pause)
                       

        
    async def land_callback(self, request, response):
        self.loop.run_until_complete(self.perform_land())
        response.connect_success = True
        return response
    
    async def perform_land(self):
        await self.drone.action.land()
        self.get_logger().info('Drone landing')

    async def turn_callback(self, request, response):
        self.start_pause.trigger = False
        self.telempause.call_async(self.start_pause)
        self.loop.run_until_complete(self.perform_turn(self.latitude, self.longitude, self.altitude, request.turn_yaw))
        response.connect_success = True
        return response
    
    async def perform_turn(self, lat,  lon, alt, yaw):
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
                                        yaw,
                                        float('nan')))
        mission_plan = MissionPlan(mission_items)
        while True:
            try:
                await self.drone.mission.upload_mission(mission_plan)
                
            except Exception as e:
                self.get_logger().error(f"Drone connection failed: {str(e)}")
                await asyncio.sleep(2)
            else:
                try:
                    await self.drone.mission.start_mission()
                except Exception as e:
                    self.get_logger().error(f"Drone connection failed: {str(e)}")
                    await asyncio.sleep(2)
                else:
                    self.get_logger().info(f'Drone going to point, altitude: {alt}, longitude: {lon}, latitude: {lat}')
                    self.start_pause.trigger = True
                    self.telempause.call_async(self.start_pause)
                    break
        

    

def main(args=None):
    rclpy.init(args=args)
    control_server = ControlServerNode()
    rclpy.spin(control_server)
    control_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
