import rclpy
from rclpy.node import Node
from transitions import Machine
from more_interfaces.srv import Comms, Arduino1, Alert, Trigger, Mission, Timer
from more_interfaces.msg import Trials1
from std_msgs.msg import String, Bool
import time
import math

abort_flag = False
patrol_flag = False
reroute_flag = False
action_todo = "idle"
iter_waypoints = 0
waypoints = []
mission_abort = False
mission_arrival_flag = False
mission_complete = False
avoid_obstacle = False
avoidwaypoints = []
mission_id = ""
in_mission = False


class MyArduinoListener(Node):
    def __init__(self, msg_reciever_node):
        super().__init__('arduino_listener')
        self.arduino_action_complete = self.create_subscription(String, "arduino_action_complete", self.arduino_callback, 10)
        self.msg_reciever_node = msg_reciever_node
    def arduino_callback(self, msg):
        global abort_flag
        global action_todo
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

        if not abort_flag:
            if self.arduino_complete_task in actions:
                action_todo = actions[self.arduino_complete_task]
            else:
                pass
        else:
            self.get_logger().info("aborting flow")
            if self.arduino_complete_task in abort_actions:
                self.current_state, action_todo = abort_actions[self.arduino_complete_task]
                abort_flag = True
   

class MyMessageReceiver(Node):
    def __init__(self):
        super().__init__('message_receiver')
        self.todo_server = self.create_service(Alert, "set_drone_action", self.set_action_callback)
        self.telempause = self.create_client(Trigger, 'start_stop_telemetry')
        self.auto_client = self.create_client(Trigger, 'auto_mode')
        self.manual_client = self.create_client(Trigger, 'manual_mode')
        self.msg, self.waypoints_received =  Trigger.Request(), None
    def set_action_callback(self, request, response):
        global mission_id
        action = request.execute_action
        mission_id = request.mission_id
       

        # Define a dictionary to map actions to corresponding functions or values
        action_mapping = {
            'abort_mission': self.handle_abort,
            'patrol': self.handle_patrol,
            'start': self.handle_start,
            'reroute': self.handle_reroute,
            'continue': self.handle_continue,
            'manual_mode': self.handle_manual_mode,
            'turn_left':  self.handle_turn_left,
            'turn_right': self.handle_turn_right,
            'close_centering': self.handle_close_centering,
            'open_centering': self.handle_open_centering,
            'raise_platform': self.handle_raise_platform,
            'lower_platform': self.handle_lower_platform,   
            'open_hatch': self.handle_open_hatch,
            'close_hatch': self.handle_close_hatch
        }
            

        # Use the dictionary to call the appropriate function or set the appropriate value
        if action in action_mapping:
            action_mapping[action](request)
       

        response.success_trigger = True
        return response
    
    def handle_open_hatch(self, request):
        global action_todo
        action_todo = "open_hatch"

    def handle_close_hatch(self, request):
        global action_todo
        action_todo = "close_hatch"

    def handle_raise_platform(self, request):
        global action_todo
        action_todo = "raise_platform"

    def handle_lower_platform(self, request):
        global action_todo
        action_todo = "lower_platform"
    
    def handle_open_centering(self, request):
        global action_todo
        action_todo = "open_centering"

    def handle_close_centering(self, request):
        global action_todo
        action_todo = "close_centering"

    def handle_turn_left(self, request):
        global action_todo
        action_todo = "turn_left"

    def handle_turn_right(self, request):
        global action_todo
        action_todo = "turn_right"

    def handle_manual_mode(self, request):
        global action_todo
        action_todo = "manual_mode"

    def handle_abort(self, request):
        global action_todo
        self.get_logger().info("abort")
        self.msg.trigger = False
        self.auto_client.call_async(self.msg)
        self.manual_client.call_async(self.msg)
        action_todo = 'abort'

    def handle_patrol(self, request):
        self.msg.trigger = True
        self.telempause.call_async(self.msg)
        self.get_logger().info("patrol")
        global iter_waypoints
        iter_waypoints = 0
        self.patrol_flag = True
        self.waypoints_received  = request.waypoints

    def handle_start(self, request):
        global waypoints
        global avoidwaypoints
        global avoid_obstacle
        global action_todo
        self.msg.trigger = True
        self.telempause.call_async(self.msg)
        start_waypoint = request.waypoints
        print(start_waypoint)
        if len(start_waypoint) == 2:   
            print('here2')
            waypoints = start_waypoint
        else:
            print('here')
            avoidwaypoints = start_waypoint
            avoid_obstacle = True

        self.get_logger().info(f'{waypoints}')
        action_todo = 'start'

    def handle_reroute(self, request):
        global waypoints
        global avoidwaypoints
        global avoid_obstacle
        global reroute_flag
        global action_todo
        self.msg.trigger = False
        self.auto_client.call_async(self.msg)
        self.manual_client.call_async(self.msg)
        start_waypoint = request.waypoints
        if len(start_waypoint) == 2:   
            waypoints = start_waypoint
        else:
            avoidwaypoints = start_waypoint
            avoidwaypoints = True
        action_todo = 'reroute'
        reroute_flag = True
        self.get_logger().info('resetting the auto and manual mode')

    def handle_continue(self, request):
        self.get_logger().info("continued")
        global action_todo
        if patrol_flag:
            self.msg.trigger = False
            self.auto_client.call_async(self.msg)
            self.manual_client.call_async(self.msg)
        else:
            self.msg.trigger = False
            self.auto_client.call_async(self.msg)
            self.manual_client.call_async(self.msg)
            action_todo = 'abort'


    def patrol_coordinates(self):
        global iter_waypoints, waypoints
        global action_todo
        global patrol_flag
        self.get_logger().info(f"Iterating through waypoints: {iter_waypoints}")
        self.get_logger().info(f"Total waypoints received: {len(self.waypoints_received)}")

        if iter_waypoints < len(self.waypoints_received):
            waypoints = [self.waypoints_received[iter_waypoints], self.waypoints_received[self.iter_waypoints + 1]]
            action_todo = 'patrol' if iter_waypoints == 0 else 'continue'
            iter_waypoints += 2
            patrol_flag = True
        else:
            patrol_flag = False
            action_todo = 'continue'


class MyMissionComplete(Node):
    def __init__(self, msg_reciever_node):
        super().__init__('mission_complete')
        self.missioncompleteserver = self.create_service(Trigger, 'mission_complete', self.missioncomplete_callback)
        self.mission_update_client = self.create_client(Mission, "mission_status")
        self.msg_reciever_node = msg_reciever_node
        self.mission_request = Mission.Request()

        
    def missioncomplete_callback(self, request, response):
        global mission_arrival_flag, mission_complete
        if mission_arrival_flag == True:
            self.get_logger().info("Drone arrived at waypoint")
            mission_complete = request.trigger
            self.mission_request.mission_status = 6.0 #sent to signal that mission completed
            global mission_id
            global action_todo
            self.mission_request.mission_id = mission_id
            self.mission_update_client.call_async(self.mission_request)
            action_todo = 'auto_mode'
            mission_arrival_flag = False
        response.success = True
        return response

        

class MyStatusUpdate(Node):
    def __init__(self, msg_reciever_node):
        super().__init__('status_update')
        self.drone_status_subscriber = self.create_subscription(Trials1, "status_update", self.drone_status_callback, 10)
        self.msg_reciever_node = msg_reciever_node
        self.flag = False
    def drone_status_callback(self, msg):
        global abort_flag
        global mission_abort
        global action_todo
        global in_mission
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
        if self.speed <= 0.05 and mission_abort == True:
            self.get_logger().info("please i want to abort")
            mission_abort = False
            self.current_state = 'uncentred'
            action_todo =  'abort'
            self.abort_flag = True
        if self.flight_mode == 5:
            self.flag = True
        elif self.flight_mode == 3 and self.flag:
            action_todo = 'centre'
            self.flag = False
        # if self.flight_mode == 3 and not abort_flag and in_mission:
        #     self.get_logger().info("Drone centering")
        #     action_todo = 'centre'
class MyTrackerDoneNode(Node):
    def __init__(self, msg_reciever_node):
        super().__init__('tracker_done')
        self.msg_reciever_node = msg_reciever_node
        self.turn_complete_server = self.create_service(Trigger, 'turn_complete', self.turn_complete_callback)
        self.tracking_done_server = self.create_service(Trigger, 'tracker_done', self.tracking_done_callback)
        self.mission_update_client = self.create_client(Mission, "mission_status")
        self.manual_client = self.create_client(Trigger, 'manual_mode')
        self.mission_request, self.msg = Mission.Request(), Trigger.Request()
        
    def tracking_done_callback(self, request, response):
        self.get_logger().info("tracking mode auto Completed")
        global patrol_flag
        global reroute_flag
        global action_todo
        self.done_tracking = request.trigger
        if patrol_flag and not(reroute_flag):
            self.get_logger().info('done mission')
            self.msg_reciever_node.patrol_coordinates()
        elif not(patrol_flag) and not(reroute_flag):
            action_todo = "continue"
        elif reroute_flag:
            reroute_flag = False
            action_todo = "continue"
        response.success = True    
        return response
    
    def turn_complete_callback(self, request, response):
        self.get_logger().info("Drone done turning")
        global action_todo
        global mission_id
        self.turn_complete = request.trigger
        self.mission_request.mission_status = 7.0 #sent to signal that system in manual mode
        self.mission_request.mission_id = mission_id
        self.mission_update_client.call_async(self.mission_request)
        self.msg.trigger = True
        self.manual_client.call_async(self.msg)
        action_todo = "manual_mode"
        response.success = True
        return response

class MyStateMachineNode(Node):
    def __init__(self, arduino_node, msg_reciever_node, mission_complete_node, status_update_node):
        super().__init__('my_state_machine_node')
        self.arduino_node = arduino_node
        self.msg_reciever_node = msg_reciever_node
        self.mission_complete_node = mission_complete_node
        self.status_update_node = status_update_node
        self.my_machine = self.initialize_state_machine()
        self.arduino_client = self.create_client(Arduino1, "send_message")
        self.drone_client = self.create_client(Comms, "drone_commands")
        self.mission_update_client = self.create_client(Mission, "mission_status")
        #self.timer_complete_service = self.create_service(Timer, 'timer_complete', self.server_callback)
        self.auto_client = self.create_client(Trigger, 'auto_mode')
        self.manual_client = self.create_client(Trigger, 'manual_mode')
        
        self.start_api_client = self.create_client(Trigger, "start_stop")
        self.timer_control_client = self.create_client(Timer, 'timer_control')
        #self.turn_complete_server = self.create_service(Trigger, 'turn_complete', self.turn_complete_callback)
        #self.tracking_done_server = self.create_service(Trigger, 'tracker_done', self.tracking_done_callback)
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
        global iter_waypoint
        global waypoints
        global reroute_flag
        self.timer = self.create_timer(0.1, self.run)
        self.msg_reciever_node.action_todo , self.current_state,  self.arduino_complete_task, self.drone_altitude, self.charging, self.patrol_flag = None, 'idle', None, 3.0, True, False
        self.drone_request, self.arduino_request, self.mission_request, self.msg  = Comms.Request(), Arduino1.Request(), Mission.Request(), Trigger.Request()
        self.speed, self.flight_mode, self.drone_state, self.iter_waypoints, self.drone_state, self.battery_level = None, None, None, iter_waypoints, 'on_ground', 0.0
        self.turn_complete, self.abort_flag, self.action_done, self.speed, self.done_tracking = None, False, None, None, False
        self.last_mission_complete, reroute_flag = False, False
        self.mission_abort, self.waypoints_received, self.system_initialize = False, None, False
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
    


    def run(self):
        state_actions = {
            'idle': self.handle_idle,
            'start': self.handle_start,
            'patrol': self.handle_start,
            'lift_platform': self.handle_lift_platform,
            'uncentre': self.handle_uncentre,
            'abort_mission': self.handle_abort_mission,
            'mission': self.handle_mission,
            'auto_mode': self.handle_auto_mode,
            'reroute': self.handle_reroute,
            'manual_mode': self.handle_manual_mode,
            'continue': self.handle_continue,
            'centre': self.handle_centre,
            'dropplatform': self.handle_drop_platform,
            'closehatch': self.handle_close_hatch,
            'idlestate': self.handle_idle_state,
            'turn_left': self.handle_turn_left,
            'turn_right': self.handle_turn_right,
            'open_hatch': self.handle_open_hatch_init,
            'close_hatch': self.handle_close_hatch_init,
            'close_centering': self.handle_close_centering_init,
            'open_centering': self.handle_open_centering_init,
            'raise_platform': self.handle_raise_platform_init,
            'lower_platform': self.handle_lower_platform_init,
        }
        global action_todo
        if action_todo in state_actions:
            state_actions[action_todo]()

    def handle_idle(self):
        self.get_logger().info('I am in idle state')
        if self.charging:
            self.arduino_command("G")
            self.charging = False
        else:
            self.get_logger().info('I am already charging')

    def handle_start(self):
        global action_todo
        if self.current_state == 'idle':
            self.open_hatch()
            self.current_state = 'hatchopened'
        else:
            self.get_logger().warn(f'Action cannot be executed whilst {self.current_state} state')
        action_todo = None    
    def handle_lift_platform(self):
        global action_todo
        global abort_flag
        if self.current_state == 'hatchopened' and not abort_flag:
            self.lift_platform()
            self.current_state = 'platformup'
        else:
            self.get_logger().warn(f'Action cannot be executed whilst {self.current_state} state')
        action_todo = None
    def handle_uncentre(self):
        global abort_flag
        global action_todo
        if self.current_state == 'platformup' and not abort_flag:
            self.uncentre()
            self.current_state = 'uncentred'
        else:
            self.get_logger().warn(f'Action cannot be executed whilst {self.current_state} state')
        action_todo = None
    
    def handle_abort_mission(self):
        global action_todo
        global mission_abort 
        global abort_flag
        if action_todo == 'abort_mission' or abort_flag:
            
            mission_abort = True

            state_functions = {
                'missiondone': (self.reverse_mission, 'aborting1'),
                'auto_mode': (self.reverse_mission, 'aborting1'),  
                'manual_mode': (self.reverse_mission, 'aborting1'),
                'turned_left': (self.reverse_mission, 'aborting1'),  
                'turned_right': (self.reverse_mission, 'aborting1'),  
                'rerouted': (self.reverse_mission, 'aborting1'),  
                'uncentred': (self.reverse_uncentering, 'aborting2'),
                'platformup': (self.reverse_liftplatform, 'aborting3'),
                'hatchopened': (self.reverse_openinghatch, 'idle')
            }

            if self.current_state in state_functions:
                function, new_state = state_functions[self.current_state]
                function()
                
                if self.current_state in ['auto_mode']:  
                    self.msg.trigger = False
                    self.auto_client.call_async(self.msg)
                if self.current_state in ['manual_mode']:
                    self.msg.trigger = False
                    self.manual_client.call_async(self.msg)

                self.current_state = new_state
                abort_flag = True if new_state != 'idle' else False
                action_todo = None

    def handle_mission(self):
        global action_todo
        global abort_flag
        if self.current_state == 'uncentred' and not abort_flag:
            self.start_mission()
            self.current_state = 'missiondone'
        else:
            self.get_logger().warn(f'Action {action_todo} cannot be executed whilst {self.current_state} state')
        action_todo = None

    def handle_auto_mode(self):
        global action_todo
        if self.current_state in  ['missiondone', 'rerouted', 'continued']:
            self.auto_mode()
            self.current_state = 'auto_mode'
        else:
            self.get_logger().warn(f'Action {action_todo} cannot be executed whilst {self.current_state} state')
        action_todo = None  

    def handle_reroute(self):
        global action_todo
        if self.current_state in ['missiondone', 'auto_mode', 'manual_mode', 'rerouted', 'turned_left', 'turned_right']:
            self.reroute()
            self.current_state = 'rerouted'
        else:
            self.get_logger().warn(f'Action {action_todo} cannot be executed whilst {self.current_state} state')
        action_todo = None

    def handle_manual_mode(self):
        print('about to go to manual')
        global action_todo
        if self.current_state in  ['auto_mode', 'turned_left', 'turned_right']:
            self.manual_mode()
            self.current_state = 'manual_mode'
        else:
            self.get_logger().warn(f'Action {action_todo} cannot be executed whilst {self.current_state} state')
        action_todo = None

    def handle_continue(self):
        global action_todo
        global patrol_flag
        if self.current_state in ["manual_mode", "auto_mode",'continued','turned_left', 'turned_right']:
            if patrol_flag == False:
                self.continue_mission()
                self.current_state = 'returned_launch'
                
            else:
                self.get_logger().info('patrol mission')
                self.continue_patrol()
                self.current_state = 'continued'
        else:
            self.get_logger().warn(f'Action {action_todo} cannot be executed whilst {self.current_state} state')
        action_todo = None
    
    def handle_centre(self):
        global action_todo
        if self.current_state == 'returned_launch':
            self.centre()
            self.current_state = "centred"
        else:
            self.get_logger().warn(f'Action {action_todo} cannot be executed whilst {self.current_state} state')
        action_todo = None
    def handle_drop_platform(self):
        global action_todo
        if self.current_state == "centred":
            self.drop_platform()
            self.current_state = 'platformdown'
    
        else:
            self.get_logger().warn(f'Action {action_todo} cannot be executed whilst {self.current_state} state')
        action_todo = None
    def handle_close_hatch(self):
        global action_todo
        if self.current_state == "platformdown":
            self.close_hatch()
            self.current_state = 'hatchclosed'
        else:
            self.get_logger().warn(f'Action {action_todo} cannot be executed whilst {self.current_state} state')
        action_todo = None

    def handle_idle_state(self):
        global action_todo
        self.mission_complete_trigger()
        self.current_state = "idle"
        action_todo = None

    def handle_turn_left(self):
        global action_todo
        if self.current_state == 'manual_mode':
            self.turn_left()
            self.current_state = 'turned_left'
        else:
            self.get_logger().warn(f'Action {action_todo} cannot be executed whilst {self.current_state} state')
        action_todo = None

    def handle_turn_right(self):
        global action_todo
        if self.current_state == 'manual_mode':
            self.turn_right()
            self.current_state = 'turned_right'

        else:
            self.get_logger().warn(f'Action cannot be executed whilst {self.current_state} state')
        action_todo = None

    def handle_open_hatch_init(self):
        global action_todo
        self.system_initialize = True
        self.open_hatch_initialize()
        self.current_state = 'hatchopened'
        action_todo = None

    def handle_close_hatch_init(self):
        global action_todo
        self.system_initialize = True
        self.close_hatch_initialize()
        self.current_state = 'hatchclosed'
        action_todo = None

    def handle_close_centering_init(self):
        global action_todo
        self.system_initialize = True
        self.close_centering_initialize()
        self.current_state = 'centred'
        action_todo = None

    def handle_open_centering_init(self):
        global action_todo
        self.system_initialize = True
        self.open_centering_initalize()
        self.current_state = 'uncentred'
        action_todo = None

    def handle_raise_platform_init(self):
        global action_todo
        self.system_initialize = True
        self.raise_platform_initialize()
        self.current_state = 'platformup'
        action_todo = None

    def handle_lower_platform_init(self):
        global action_todo
        self.system_initialize = True
        self.lower_platform_initialize()
        self.current_state = 'platformdown'
        action_todo = None

            
    def action_hatchopening(self):

        self.get_logger().info('Hatch opening...')
        if self.system_initialize:
            self.arduino_request.trigger = True
            self.system_initialize = False
        else:
            self.start_api_node(True)
            self.update_mission_status(3.0) #mission started confirmation

        self.arduino_command("S")  #stop charging
        time.sleep(2)
        self.arduino_command("H")   #hatch opening

    def start_api_node(self, bool_val):
        """Start the API node."""
        self.msg.trigger = bool_val
        self.start_api_client.call_async(self.msg)
    
    def update_mission_status(self, command):
        '''Send confirmation to backend of mission status'''
        self.mission_request.mission_status = command 
        global mission_id
        self.mission_request.mission_id = mission_id
        self.mission_update_client.call_async(self.mission_request)


    def arduino_command(self, command):
        """Commands the arduino to do an action"""
        global mission_id
        self.arduino_request.mission_id = mission_id
        self.arduino_request.message = command  
        self.arduino_client.call_async(self.arduino_request)


    def action_liftplatform(self):
        '''Platform raising'''
        if self.system_initialize :
            self.system_initialize = False
            self.arduino_request.trigger = True
        else:
            self.arduino_request.trigger = False

        self.arduino_command("R")  #Raise platform

    def action_uncentering(self):
        self.get_logger().info('Drone uncentering....')
        if self.system_initialize == True:
            self.system_initialize = False
            self.arduino_request.trigger = True
        else:
            self.arduino_request.trigger = False
        self.arduino_command("U")  #Drone uncentering

    def action_continue_mission(self):
        self.update_mission_status(8.0) #signal to backend that system continued
        self.manual_auto_left_right(self.manual_client, False)
        self.manual_auto_left_right(self.auto_client, False)
        self.return_to_launch()
    
    def manual_auto_left_right(self, client_name, bool_val):
        self.msg.trigger = bool_val
        client_name.call_async(self.msg)

    def action_continue_patrol(self):
        self.update_mission_status(8.0) #signal to backend that system continued
        self.manual_auto(self.manual_client, False)
        
    def action_turn_right(self):
        self.manual_auto_left_right(self.turn_right_client, True)
        print("turning right")

    def action_turn_left(self):
        self.manual_auto_left_right(self.turn_left_client, True)
    
    def action_mission(self):
        self.get_logger().info('Drone going to coordinates')
        global avoid_obstacle
        if avoid_obstacle:
             self.drone_avoidcommands(self.sendmission_client, 'Goto')
        else:
            self.drone_commands(self.sendmission_client, "Goto")

    def action_patrol(self):
        self.get_logger().info('Drone on Patrol')
        self.update_mission_status(8.0)
        self.drone_commands(self.sendmission_client, "Goto")

    def action_returning(self):
        self.get_logger().info('Reversing mission')
        self.drone_commands(self.missionclear_client, "Clearmission")
        time.sleep(2)
        self.drone_request.message = 'Goback'
        self.drone_request.trigger = True
        self.return_client.call_async(self.drone_request)

    def action_land(self):
        self.get_logger().info('Drone landing onto the base station')
        self.drone_commands(self.land_client, "Land")

    def action_centre(self):
        self.get_logger().info('Drone is being centred')
        if self.system_initialize == True:
            self.system_initialize = False
            self.arduino_request.trigger = True
        else:
            self.arduino_request.trigger = False
        self.arduino_command("C")  #Drone uncentering
    
    def action_dropplatform(self):
        self.get_logger().info('Station platforming dropping')
        if self.system_initialize == True:
            self.system_initialize = False
            self.arduino_request.trigger = True
        else: 
            self.arduino_request.trigger = False
            global mission_id
        self.arduino_request.mission_id = mission_id
        self.arduino_command("D")  #Drone uncentering

    def action_closehatch(self):
        self.get_logger().info('closing hatch')
        if self.system_initialize == True:
            self.system_initialize = False
            self.arduino_request.trigger = True
            
        else: 
            self.arduino_request.trigger = False
            global mission_id
        self.arduino_request.mission_id = mission_id
        self.arduino_command("T") #closing hatch

    def action_missioncomplete(self):
        self.get_logger().info('mission completed')
        global in_mission
        #send confirmation that mission is complete
        self.update_mission_status(4.0) #mission completed confirmation
        #stops the api node from posting to backend
        self.start_api_node(False)
        in_mission = False
        #self.telempause.call_async(self.msg


    def action_reroute(self):
        self.get_logger().info('Drone rerouting ...')
        self.update_mission_status(8.0) #signal to backend that system rerouting
        self.get_logger().info('Drone rerouting now')
        global avoidwaypoints
        global avoid_obstacle
        if avoid_obstacle:
            self.drone_avoidcommands(self.reroute_client, 'reroute')
        else:
            self.drone_commands(self.reroute_client, 'reroute')
        global mission_arrival_flag
        mission_arrival_flag = True
        
    def drone_avoidcommands(self, client_name, command):
        global avoid_obstacle
        global avoidwaypoints
        self.drone_request.trigger = True
        self.drone_request.message = command
        self.drone_request.avoid_obstacle_waypoints = avoidwaypoints 
        self.drone_request.pos_altitude = str(self.drone_altitude)
        client_name.call_async(self.drone_request)
        self.drone_request.trigger = True
        waypoints_comp = avoidwaypoints[-2:]
        self.drone_request.pos_longitude = waypoints_comp[0]
        self.drone_request.pos_latitude = waypoints_comp[1]
        self.send_mission_coordinates.call_async(self.drone_request)
        global mission_arrival_flag
        mission_arrival_flag = True
        avoid_obstacle = False
    
    def drone_commands(self, client_name, command):
        global waypoints
        self.get_logger().info(f'{self.iter_waypoints}')
        self.get_logger().info(f'waypoints:{waypoints}')
        self.drone_request.trigger = False
        self.drone_request.message = command
        self.drone_request.pos_longitude = waypoints[self.iter_waypoints]
        self.drone_request.pos_latitude = waypoints[self.iter_waypoints+1]
        self.drone_request.pos_altitude = str(self.drone_altitude)
        client_name.call_async(self.drone_request)
        self.send_mission_coordinates.call_async(self.drone_request)
        global mission_arrival_flag
        mission_arrival_flag = True
        in_mission = True
        

    def action_automode(self):
        self.get_logger().info('I am in auto mode ..')
        self.manual_auto_left_right(self.auto_client, True)

    def action_manual_mode(self):
        self.get_logger().info("Initiating manual mode")
        self.manual_auto_left_right(self.auto_client, False)
        self.update_mission_status(7.0) #signal to backend that system manual mode 
        global mission_id 
        self.msg.mission_id = mission_id
        self.drone_commands(self.missionclear_client, "Clearmission")
        self.manual_auto_left_right(self.manual_client, True)

def main(args=None):
    rclpy.init(args=args)
    msg_reciever_node = MyMessageReceiver()
    tracker_node = MyTrackerDoneNode(msg_reciever_node)
    arduino_node = MyArduinoListener(msg_reciever_node)
    mission_complete_node = MyMissionComplete(msg_reciever_node)
    status_update_node = MyStatusUpdate(msg_reciever_node)
    state_machine_node = MyStateMachineNode(arduino_node, msg_reciever_node, mission_complete_node, status_update_node)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(arduino_node)
    executor.add_node(msg_reciever_node)
    executor.add_node(tracker_node)
    executor.add_node(mission_complete_node)
    executor.add_node(status_update_node)
    executor.add_node(state_machine_node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        arduino_node.destroy_node()
        msg_reciever_node.destroy_node()
        tracker_node.destroy_node()
        mission_complete_node.destroy_node()
        status_update_node.destroy_node()
        state_machine_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

        
