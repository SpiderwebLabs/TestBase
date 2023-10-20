import rclpy
from rclpy.node import Node
from transitions import Machine
from more_interfaces.srv import Comms, Arduino1, Alert, Trigger, Mission, Timer
from more_interfaces.msg import Trials1
from std_msgs.msg import String, Bool
import time
import math



class MyArduinoListener(Node):
    def __init__(self):
        super().__init__('arduino_listener')
        self.arduino_action_complete = self.create_subscription(String, "arduino_action_complete", self.arduino_callback, 10)

    def arduino_callback(self, msg):
        self.arduino_complete_task = msg.data
        self.get_logger().info(f"{self.arduino_complete_task}")

        actions = {
            'Hatchopened': 'lift_platform',
            'Platformup': 'uncentre',
            'Uncentred': 'mission',
            'Centred': 'dropplatform',
            'Platformdown': 'closehatch',
            'Hatchclosed': 'idlestate'
        }

        abort_actions = {
            "Centred": ('platformup', 'abort'),
            "Platformdown": ('hatchopened', 'abort'),
            'Hatchclosed': ('idle', 'abort')
        }

        if not self.abort_flag:
            if self.arduino_complete_task in actions:
                self.action_todo = actions[self.arduino_complete_task]
            else:
                pass
        else:
            self.get_logger().info("aborting flow")
            if self.arduino_complete_task in abort_actions:
                self.current_state, self.action_todo = abort_actions[self.arduino_complete_task]
                self.abort_flag = True

    
    
        

class MyMessageReceiver(Node):
    def __init__(self):
        super().__init__('message_receiver')
        self.todo_server = self.create_service(Alert, "set_drone_action", self.set_action_callback)
    def set_action_callback(self, request, response):
        action = request.execute_action
        self.get_logger().info(f"{self.action_todo}")
        self.mission_id = request.mission_id

        # Define a dictionary to map actions to corresponding functions or values
        action_mapping = {
            'abort': self.handle_abort,
            'patrol': self.handle_patrol,
            'start': self.handle_start,
            'reroute': self.handle_reroute,
            'continue': self.handle_continue
        }

        # Use the dictionary to call the appropriate function or set the appropriate value
        if action in action_mapping:
            action_mapping[action](request)

        response.success_trigger = True
        return response

    def handle_abort(self, request):
        self.get_logger().info("abort")
        self.msg.trigger = False
        self.auto_client.call_async(self.msg)
        self.manual_client.call_async(self.msg)
        self.action_todo = 'abort'

    def handle_patrol(self, request):
        self.msg.trigger = True
        self.telempause.call_async(self.msg)
        self.get_logger().info("patrol")
        self.iter_waypoints = 0
        self.patrol_flag = True
        self.waypoints = request.waypoints

    def handle_start(self, request):
        self.msg.trigger = True
        self.telempause.call_async(self.msg)
        self.waypoints = request.waypoints
        self.action_todo = 'start'

    def handle_reroute(self, request):
        self.waypoints_received = request.waypoints
        self.waypoints = self.waypoints_received
        self.action_todo = 'reroute'
        self.reroute_flag = True
        self.get_logger().info('resetting the auto and manual mode')

    def handle_continue(self, request):
        if self.patrol_flag:
            self.get_logger().info("continued")
        else:
            self.msg.trigger = False
            self.auto_client.call_async(self.msg)
            self.manual_client.call_async(self.msg)
            self.action_todo = 'abort'


    def patrol_coordinates(self):
        self.get_logger().info(f"{self.iter_waypoints}")
        self.get_logger().info(f"{self.waypoints_received}")
        self.get_logger().info(f"{len(self.waypoints_received)}")
        if self.iter_waypoints != len(self.waypoints_received):
            self.waypoints = [self.waypoints_received[self.iter_waypoints], self.waypoints_received[self.iter_waypoints+1]]
            self.get_logger().info(f'{self.waypoints}')
            if self.iter_waypoints == 0:
                self.action_todo = 'patrol'
                self.patrol_flag = True
                self.iter_waypoints +=2
      
            elif self.iter_waypoints != 0:
                self.action_todo = 'continue'
                self.patrol_flag = True
                self.iter_waypoints +=2
        else:
            self.patrol_flag = False
            self.action_todo = 'continue'

class MyMissionComplete(Node):
    def __init__(self):
        super().__init__('mission_complete')
        self.missioncompleteserver = self.create_service(Trigger, 'mission_complete', self.missioncomplete_callback)
    def missioncomplete_callback(self, request, response):
        if self.mission_arrival_flag == True:
            self.get_logger().info("Drone arrived at waypoint")
            self.mission_complete = request.trigger
            self.mission_request.mission_status = 6.0 #sent to signal that mission completed
            self.mission_request.mission_id = self.mission_id
            self.mission_update_client.call_async(self.mission_request)
            self.action_todo = 'auto_mode'
            self.mission_arrival_flag = False
        response.success = True
        return response

        

class MyStatusUpdate(Node):
    def __init__(self):
        super().__init__('status_update')
        self.drone_status_subscriber = self.create_subscription(Trials1, "status_update", self.drone_status_callback, 10)

    def drone_status_callback(self, msg):
        '''This function is called when the server receives a message from the drone node'''
        self.battery_level = msg.battery_percentage # need to change this to the actual battery level
        if msg.armed == True:
            self.drone_state = "Armed"
        velocity_x = msg.velocity_x
        velocity_y = msg.velocity_y
        velocity_z = msg.velocity_z
        self.speed = math.sqrt((velocity_x)**2 + (velocity_y)**2 + (velocity_z)**2)
        self.drone_state = msg.landed_state
        self.flight_mode = msg.flight_mode
        self.get_logger().info(f"flight_mode: {self.flight_mode}")
        if self.speed <= 0.05 and self.mission_abort == True:
            self.get_logger().info("please i want to abort")
            self.mission_abort = False
            self.current_state = 'uncentred'
            self.action_todo =  'abort'
            self.abort_flag = True
        if self.flight_mode == 5 and not self.abort_flag:
            self.action_todo = 'centre'

class MyStateMachineNode(Node):
    def __init__(self, arduino_node, msg_reciever_node, mission_complete_node, status_update_node):
        super().__init__('my_state_machine_node')
        self.arduino_node = arduino_node
        self.msg_reciever_node = msg_reciever_node
        self.mission_complete_node = mission_complete_node
        self.status_update_node = status_update_node
        self.my_machine = self.initialize_state_machine()

        self.arduino_client = self.create_client(Arduino1, "send_message")
        self.auto_client = self.create_client(Trigger, 'auto_mode')
        self.manual_client = self.create_client(Trigger, 'manual_mode')
        self.drone_client = self.create_client(Comms, "drone_commands")
        self.mission_update_client = self.create_client(Mission, "mission_status")
        #self.timer_complete_service = self.create_service(Timer, 'timer_complete', self.server_callback)
       
        
        self.start_api_client = self.create_client(Trigger, "start_stop")
        self.timer_control_client = self.create_client(Timer, 'timer_control')
        self.turn_complete_server = self.create_service(Trigger, 'turn_complete', self.turn_complete_callback)
        self.tracking_done_server = self.create_service(Trigger, 'tracker_done', self.tracking_done_callback)
        self.send_mission_coordinates = self.create_client(Comms, 'get_coordinates')
        
        #communication to drone clients
        self.arm_client = self.create_client(Comms, 'arm')
        self.takeoff_client = self.create_client(Comms, 'takeoff')
        self.sendmission_client = self.create_client(Comms, 'goto')
        self.land_client = self.create_client(Comms, 'land')
        self.return_client = self.create_client(Comms, 'returnback')
        self.missionclear_client = self.create_client(Comms, 'clear_mission')
        self.pausemission_client = self.create_client(Comms, 'pause_mission')
        self.Coordinates_client = self.create_client(Comms, 'get_coordinates')
        self.reroute_client = self.create_client(Comms, 'reroute')
        self.turn_right_client = self.create_client(Trigger, 'turn_right')
        self.turn_left_client = self.create_client(Trigger, 'turn_left')
        self.telempause = self.create_client(Trigger, 'start_stop_telemetry')
        self.timer = self.create_timer(0.1, self.run)
        self.action_todo , self.current_state,  self.arduino_complete_task, self.drone_altitude, self.charging, self.patrol_flag = None, 'idle', None, 3.0, True, False
        self.drone_request, self.arduino_request, self.mission_request, self.msg  = Comms.Request(), Arduino1.Request(), Mission.Request(), Trigger.Request()
        self.speed, self.waypoints, self.flight_mode, self.drone_state, self.iter_waypoints, self.mission_complete, self.drone_state, self.battery_level = None, '', None, None, 0, False, 'on_ground', 0.0
        self.turn_complete, self.abort_flag, self.action_done, self.speed, self.done_tracking = None, False, None, None, False
        self.last_mission_complete, self.mission_id, self.reroute_flag = False, "", False
        self.mission_abort, self.waypoints_received, self.system_initialize, self.mission_arrival_flag = False, None, False, False
    def initialize_state_machine(self):
        states = ['idle', 'hatchopened', 'platformup', 'uncentred', 'missiondone', 'returned_launch', 
                'landed', 'centred', 'platformdown', 'hatchclosed', 'missioncompleted', 'rerouted', 'auto_mode', 'manual_mode', 'continued', 'turned_left', 'turned_right', 'aborting1', 'abortcalled', 'aborting2', 'aborting3']
        transitions = [
            {'trigger': 'open_hatch', 'source': 'idle', 'dest': 'hatchopened', 'after': 'action_hatchopening'},
            {'trigger': 'lift_platform', 'source': 'hatchopened', 'dest': 'platformup', 'after': 'action_liftplatform'},
            {'trigger': 'uncentre', 'source': 'platformup', 'dest': 'uncentred', 'after': 'action_uncentering'},
            {'trigger': 'start_mission', 'source': 'uncentred', 'dest': 'missiondone', 'after': 'action_mission'},
            {'trigger': 'return_to_launch', 'source': 'missiondone', 'dest': 'returned_launch', 'before': 'action_returning'},
            {'trigger': 'drone_land', 'source': 'returned_launch', 'dest': 'landed', 'before': 'action_land'},
            {'trigger': 'centre', 'source': ['landed', 'returned_launch'], 'dest': 'centred', 'before': 'action_centre'},
            {'trigger': 'drop_platform', 'source': ['missiondone','centred'], 'dest': 'platformdown', 'before': 'action_dropplatform'},
            {'trigger': 'close_hatch', 'source': 'platformdown', 'dest': 'hatchclosed', 'before': 'action_closehatch'},
            {'trigger': 'auto_mode', 'source': ['missiondone', 'rerouted'], 'dest': 'auto_mode', 'before': 'action_automode'},
            {'trigger': 'reroute', 'source': '*', 'dest': 'rerouted', 'before': 'action_reroute'},
            {'trigger': 'manual_mode', 'source': ['auto_mode', 'turned_right', 'turned_left'], 'dest': 'manual_mode', 'after': 'action_manual_mode'},
            {'trigger': 'turn_left', 'source': ['manual_mode', 'turned_left', 'turned_right'], 'dest': 'turned_left', 'before': 'action_turn_left'},
            {'trigger': 'turn_right', 'source': ['manual_mode', 'turned_left', 'turned_right'], 'dest': 'turned_right', 'before': 'action_turn_right'},
            {'trigger': 'continue_mission', 'source': ['manual_mode', 'auto_mode', 'missiondone', 'turned_left', 'turned_right'], 'dest': 'missiondone', 'after': 'action_continue_mission'},
            {'trigger': 'continue_patrol', 'source': ['manual_mode', 'auto_mode', 'missiondone', 'turned_left', 'turned_right'], 'dest': 'missiondone', 'after': 'action_patrol' },
            {'trigger': 'mission_complete_trigger', 'source': ['idle', 'hatchclosed', 'auto_mode'], 'dest': 'idle', 'before': 'action_missioncomplete'},
            {'trigger': 'reverse_openinghatch', 'source': ['hatchopened', 'aborting3'], 'dest': 'idle', 'after': 'action_closehatch'},
            {'trigger': 'reverse_liftplatform', 'source': ['platformup', 'aborting2'], 'dest': 'aborting3', 'after': 'action_dropplatform'},
            {'trigger': 'reverse_uncentering', 'source': ['uncentred', 'aborting1'], 'dest': 'aborting2', 'after': 'action_centre'},
            {'trigger': 'reverse_mission', 'source': ['missiondone', 'auto_mode', 'manual_mode', 'rerouted', 'turned_left', 'turned_right'], 'dest': 'aborting1', 'after': 'action_returning'},
            {'trigger': 'open_hatch_initialize', 'source': '*', 'dest': 'hatchopened', 'before': 'action_hatchopening'},
            {'trigger': 'close_hatch_initialize', 'source': '*', 'dest': 'hatchclosed', 'before': 'action_closehatch' },
            {'trigger': 'close_centering_initialize', 'source': '*', 'dest': 'centred', 'before': 'action_centre'},
            {'trigger': 'open_centering_initialize', 'source': '*', 'dest': 'uncentred', 'before': 'action_uncentering'},
            {'trigger': 'raise_platform_initialize', 'source': '*', 'dest': 'platformup', 'before': 'action_liftplatform'},
            {'trigger': 'raise_plaform_initialize', 'source': '*', 'dest': 'platformdown', 'before': 'action_dropplatform'}
        ]
        self.current_state = 'idle'
        return Machine(model=self, states=states, transitions=transitions, initial=self.current_state)
    
    def tracking_done_callback(self, request, response):
        self.get_logger().info("tracking mode auto Completed")
        self.done_tracking = request.trigger
        if self.patrol_flag == True and not(self.reroute_flag):
            self.get_logger().info('done mission')
            self.patrol_coordinates()
        elif not(self.patrol_flag) and not(self.reroute_flag):
            self.action_todo = "continue"
        elif self.reroute_flag:
            self.reroute_flag = False
        response.success = True    
        return response

    
    def turn_complete_callback(self, request, response):
        self.get_logger().info("Drone done turning")
        self.turn_complete = request.trigger
        self.mission_request.mission_status = 7.0 #sent to signal that system in manual mode
        self.mission_request.mission_id = self.mission_id
        self.mission_update_client.call_async(self.mission_request)
        self.msg.trigger = True
        self.manual_client.call_async(self.msg)
        self.action_todo = "manual_mode"
        response.success = True
        return response
    
    def action_hatchopening(self):
        self.get_logger().info('Hatch opening...')
        if self.system_initialize == True:
            self.system_initialize = False
            self.arduino_request.trigger = True
            self.system_initialize = False
        else: 
            self.arduino_request.trigger = False
            #start the api node
            self.msg.trigger = True
            self.start_api_client.call_async(self.msg)
            #send confirmation to the backend that mission has been started
            self.mission_request.mission_status = 3.0 #sent to signal that mission in progress
            self.mission_request.mission_id = self.mission_id
            self.mission_update_client.call_async(self.mission_request)

        #stop drone charging
        self.arduino_request.message = "S"
        self.arduino_client.call_async(self.arduino_request)
        #sleep a bit 
        time.sleep(2)
        #open the hatch
        self.arduino_request.mission_id = self.mission_id
        self.arduino_request.message = "H"   #open hatch 
        self.arduino_client.call_async(self.arduino_request)

        

    def action_liftplatform(self):
        self.get_logger().info('Platform raising ...')
        if self.system_initialize == True:
            self.system_initialize = False
            self.arduino_request.trigger = True
        else:
            self.arduino_request.trigger = False

        self.arduino_request.message = "R"
        self.arduino_request.mission_id = self.mission_id
        self.arduino_client.call_async(self.arduino_request)  # raising platform
        

    def action_uncentering(self):
        self.get_logger().info('Drone uncentering....')
        if self.system_initialize == True:
            self.system_initialize = False
            self.arduino_request.trigger = True
        else:
            self.arduino_request.trigger = False
        self.arduino_request.message = "U"
        self.arduino_request.mission_id = self.mission_id
        self.arduino_client.call_async(self.arduino_request)  # uncentering drone

    def action_continue_mission(self):
        self.mission_request.mission_status = 8.0 #sent to signal that system continued
        self.mission_request.mission_id = self.mission_id
        self.mission_update_client.call_async(self.mission_request)
        self.msg.trigger = False
        self.manual_client.call_async(self.msg)
        self.return_to_launch()

    def action_continue_patrol(self):
        self.mission_request.mission_status = 8.0 #sent to signal that system icontinued
        self.mission_request.mission_id = self.mission_id
        self.mission_update_client.call_async(self.mission_request)
        self.msg.trigger = False
        self.manual_client.call_async(self.msg)
        
    def action_turn_right(self):
        self.msg.trigger = True
        self.turn_right_client.call_async(self.msg)

    def action_turn_left(self):
        self.msg.trigger = True
        self.turn_left_client.call_async(self.msg)
        

    def action_mission(self):
        self.get_logger().info('Drone going to coordinates')
        self.drone_request.message = "Goto"
        self.drone_request.trigger = True
        self.drone_request.pos_longitude = self.waypoints[0]
        self.drone_request.pos_latitude = self.waypoints[1]
        self.drone_request.pos_altitude = str(self.drone_altitude)
        self.mission_arrival_flag = True
        self.sendmission_client.call_async(self.drone_request)
        self.send_mission_coordinates.call_async(self.drone_request)

    def action_patrol(self):
        self.get_logger().info('Drone on Patrol')
        self.get_logger().info('Drone going to coordinates')
        self.mission_request.mission_status = 8.0 #sent to signal that system continued
        self.mission_request.mission_id = self.mission_id
        self.mission_update_client.call_async(self.mission_request)
        self.drone_request.message = "Goto"
        self.drone_request.trigger = True
        self.drone_request.pos_longitude = self.waypoints[0]
        self.drone_request.pos_latitude = self.waypoints[1]
        self.drone_request.pos_altitude = str(self.drone_altitude)
        self.mission_arrival_flag = True
        self.sendmission_client.call_async(self.drone_request)
        self.send_mission_coordinates.call_async(self.drone_request)

    def action_returning(self):
        self.get_logger().info('Drone returning to launch')
        self.get_logger().info('Reversing mission')
        self.drone_request.message = "Clearmission"  #clearing any uploaded mission
        self.drone_request.trigger=True
        self.missionclear_client.call_async(self.drone_request)
        time.sleep(2)
        self.drone_request.message = "Goback"
        self.drone_request.trigger = True
        self.return_client.call_async(self.drone_request)

    def action_land(self):
        self.get_logger().info('Drone landing onto the base station')
        self.drone_request.message = "Land"
        self.drone_request.trigger = True
        self.land_client.call_async(self.drone_request)

    def action_centre(self):
        self.get_logger().info('Drone is being centred')
        if self.system_initialize == True:
            self.system_initialize = False
            self.arduino_request.trigger = True
        else:
            self.arduino_request.trigger = False
        
        self.arduino_request.message = "C"
        self.arduino_request.mission_id = self.mission_id
        self.arduino_client.call_async(self.arduino_request)
    
    def action_dropplatform(self):
        self.get_logger().info('Station platforming dropping')
        if self.system_initialize == True:
            self.system_initialize = False
            self.arduino_request.trigger = True
        else: 
            self.arduino_request.trigger = False
        self.arduino_request.mission_id = self.mission_id
        self.arduino_request.message = "D"
        self.arduino_client.call_async(self.arduino_request)

    def action_closehatch(self):
        self.get_logger().info('closing hatch')
        if self.system_initialize == True:
            self.system_initialize = False
            self.arduino_request.trigger = True
            
        else: 
            self.arduino_request.trigger = False
        self.arduino_request.mission_id = self.mission_id
        self.arduino_request.message = "T"
        self.arduino_client.call_async(self.arduino_request)

    def action_missioncomplete(self):
        self.get_logger().info('mission completed')
        #send confirmation that mission is complete
        self.mission_request.mission_status = 4.0 #sent to signal that mission complete
        self.mission_request.mission_id = self.mission_id
        self.mission_update_client.call_async(self.mission_request)
        #stops the api node from posting to backend
        self.msg.trigger = False
        self.start_api_client.call_async(self.msg)
        self.msg.trigger = False
        self.telempause.call_async(self.msg)
    

    def action_reroute(self):
        self.get_logger().info('Drone rerouting ...')
        self.mission_request.mission_status = 8.0 #sent to signal that system continued
        self.mission_request.mission_id = self.mission_id
        self.mission_update_client.call_async(self.mission_request)
        # self.drone_request.message = "Clearmission"  #clearing any uploaded mission
        # self.drone_request.trigger=True
        # self.missionclear_client.call_async(self.drone_request)
        # time.sleep(3)
        self.get_logger().info('Drone rerouting now')
        self.drone_request.message = 'reroute'
        self.drone_request.pos_longitude = self.waypoints[self.iter_waypoints]
        self.drone_request.pos_latitude = self.waypoints[self.iter_waypoints+1]
        self.drone_request.pos_altitude = str(self.drone_altitude)
        self.reroute_client.call_async(self.drone_request)
        self.send_mission_coordinates.call_async(self.drone_request)
        self.mission_arrival_flag = True

    def action_automode(self):
        self.get_logger().info('I am in auto mode ..')
        self.msg.trigger = True
        self.auto_client.call_async(self.msg)

    def action_manual_mode(self):
        self.get_logger().info("Initiating manual mode")
        self.msg.trigger = False
        self.auto_client.call_async(self.msg)
        self.mission_request.mission_status = 7.0 #sent to signal that system in manual mode
        self.mission_request.mission_id = self.mission_id
        self.mission_update_client.call_async(self.mission_request)
        self.msg.trigger = True
        self.msg.mission_id = self.mission_id
        self.manual_client.call_async(self.msg)

    def run(self):
        if self.current_state == 'idle':
            self.get_logger().info('I am in idle state')
            if self.charging:
                self.arduino_request.message = 'G'
                self.arduino_client.call_async(self.arduino_request)
                self.charging = False


        if self.action_todo is not None:
            self.battery_level = 0.7
            # Do a battery check before allowing mission to start
            if self.battery_level > 0.6:
                self.transition_states()
            else:
                '''sends battery low notification to backend or rather system not ready for mission'''
                self.mission_request.mission_status = 2.0 # sent to signal that mission failed
                self.mission_request.mission_id = self.mission_id
                self.mission_update_client.call_async(self.mission_request)

    def transition_states(self):
        self.get_logger().info(f"{self.current_state}")
        self.get_logger().info(f"{self.arduino_complete_task }")
        if self.action_todo in ['start', 'patrol']:
            if self.current_state == 'idle':
                self.open_hatch()
                self.current_state = 'hatchopened'
                self.action_todo = None

        elif self.action_todo == 'lift_platform' and not(self.abort_flag):
            if self.current_state == 'hatchopened':
                self.lift_platform()
                self.current_state = 'platformup'
                self.action_todo = None

        elif  self.action_todo == 'uncentre' and not(self.abort_flag):
            if self.current_state == 'platformup' :
                self.uncentre()
                self.current_state = 'uncentred'
                self.action_todo = None
        
        elif self.action_todo == 'abort_mission' or self.abort_flag == True:
            self.mission_request.mission_status = 8.0 #sent to signal that system continued
            self.mission_request.mission_id = self.mission_id
            self.mission_update_client.call_async(self.mission_request)
            if self.current_state in ['missiondone', 'auto_mode', 'manual_mode', 'turned_left', 'turned_right', 'rerouted']:
                if self.current_state in ['automode']:
                    self.msg.trigger = False
                    self.auto_client.call_async(self.msg)
                if self.current_state in ['manual_mode']:
                    self.msg.trigger = False
                    self.manual_client.call_async(self.msg)
                self.reverse_mission()
                self.mission_abort = True
                self.current_state = 'aborting1'
                self.abort_flag = True
                self.action_todo = None
            elif self.current_state == 'uncentred':
                self.reverse_uncentering()
                self.current_state = 'aborting2'
                self.abort_flag = True
                self.action_todo = None
            elif self.current_state == 'platformup':
                self.reverse_liftplatform()
                self.current_state = "aborting3"
                self.abort_flag = True
                self.action_todo = None
            elif self.current_state == "hatchopened":
                self.reverse_openinghatch()
                self.current_state = "idle"
                self.abort_flag = False 
                self.action_todo = None
            else:
                pass
            

        # elif  (self.action_todo == 'mission' or self.action_todo == 'patrol') and not(self.abort_flag):
        elif  self.action_todo == 'mission' and not(self.abort_flag):
            self.get_logger().info(f"{self.waypoints}")
            if self.current_state == 'uncentred':
                self.start_mission()
                self.current_state = 'missiondone'
                self.action_todo = None

        elif self.action_todo == 'auto_mode':
            self.get_logger().info("i am activating automode")
            if self.current_state in  ['missiondone', 'rerouted', 'continued']:
                self.auto_mode()
                self.current_state = 'auto_mode'
                self.action_todo = None

        elif self.action_todo == 'reroute' and not(self.abort_flag):
            self.get_logger().info("rerouting")
            if self.current_state in ['missiondone', 'auto_mode', 'manual_mode', 'rerouted', 'turned_left', 'turned_right']:
                self.reroute()
                self.current_state = 'rerouted'
                self.action_todo = None

        elif self.action_todo == 'manual_mode' and not(self.abort_flag):
            if self.current_state in  ['auto_mode', 'turned_left', 'turned_right']:
                self.manual_mode()
                self.current_state = 'manual_mode'
                self.action_todo = None

        elif self.action_todo == 'continue' and not(self.abort_flag):
            if self.current_state in ["manual_mode", "auto_mode",'continued','turned_left', 'turned_right']:
                if self.patrol_flag == False:
                    self.continue_mission()
                    self.current_state = 'returned_launch'
                    self.action_todo = None
                else:
                    self.get_logger().info('patrol mission')
                    self.continue_patrol()
                    self.current_state = 'continued'
                    self.action_todo = None

        elif self.action_todo == "centre" and self.speed <= 0.05:
            if self.current_state == 'returned_launch':
                self.centre()
                self.current_state = "centred"
                self.action_todo = None
        
        elif self.action_todo == "dropplatform":
            if self.current_state == "centred":
                self.drop_platform()
                self.current_state = 'platformdown'
                self.action_todo = None
        
        elif self.action_todo == "closehatch":
            if self.current_state == "platformdown":
                self.close_hatch()
                self.current_state = 'hatchclosed'
                self.action_todo = None
        
        elif self.action_todo == 'idlestate':
            self.mission_complete_trigger()
            self.current_state = "idle"
            self.action_todo = None

        elif self.action_todo == 'turn_left' and not(self.abort_flag):
            if self.current_state == 'manual_mode':
                self.turn_left()
                self.current_state = 'turned_left'
                self.action_todo = None

        elif self.action_todo == "turn_right" and not(self.abort_flag):
            if self.current_state == 'manual_mode':
                self.turn_right()
                self.current_state = 'turned_right'
                self.action_todo = None
            
        elif self.action_todo == 'open_hatch':
            self.system_initialize = True
            self.open_hatch_initialize()
            self.current_state = 'hatchopened'
            self.action_todo = None

        elif self.action_todo == 'close_hatch':
            self.system_initialize = True
            self.close_hatch_initialize()
            self.current_state = 'hatchclosed'
            self.action_todo = None
        
        elif self.action_todo == 'close_centering':
            self.system_initialize = True
            self.close_centering_initialize()
            self.current_state = 'centred'
            self.action_todo = None
            
        elif self.action_todo == 'open_centering':
            self.system_initialize = True
            self.open_centering_initalize()
            self.current_state = 'uncentred'
            self.action_todo = None
        
        elif self.action_todo == 'raise_platform':
            self.system_initialize = True
            self.raise_platform_initialize()
            self.current_state = 'platformup'
            self.action_todo = None

        elif self.action_todo == 'lower_platform':
            self.system_initialize = True
            self.lower_platform_initialize()
            self.current_state = 'platformdown'
            self.action_todo = None
        #self.action_todo = None
        


def main(args=None):
    rclpy.init(args=args)
    arduino_node = MyArduinoListener()
    msg_reciever_node = MyMessageReceiver()
    mission_complete_node = MyMissionComplete()
    status_update_node = MyStatusUpdate()
    state_machine_node = MyStateMachineNode(arduino_node, msg_reciever_node, mission_complete_node, status_update_node)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(arduino_node)
    executor.add_node(msg_reciever_node)
    executor.add_node(mission_complete_node)
    executor.add_node(status_update_node)
    executor.add_node(state_machine_node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        arduino_node.destroy_node()
        msg_reciever_node.destroy_node()
        mission_complete_node.destroy_node()
        status_update_node.destroy_node()
        state_machine_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

        
