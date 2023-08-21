import rclpy
from rclpy.node import Node
from more_interfaces.srv import Timer, Comms, Trigger, Mission
from more_interfaces.msg import Trials1


class ManualMode(Node):
    def __init__(self):
        super().__init__('manual_node')
        self.manual_server = self.create_service(Trigger, 'manual_mode', self.manual_mode_callback)
        self.turn_client = self.create_client(Comms, 'turn')
        self.turn_right_server = self.create_service(Trigger, 'turn_right', self.turn_right_callback)
        self.turn_left_server = self.create_service(Trigger, 'turn_left', self.turn_left_callback)
        self.turn_complete_client = self.create_client(Trigger, 'turn_complete')
        self.tracking_done_client = self.create_client(Trigger, 'tracker_done')
        self.mission_update_client = self.create_client(Mission, "mission_status")
        self.mission_update_request, self.drone_request = Mission.Request(), Comms.Request()
        self.current_yaw, self.tracker_done , self.timer_request = None, Trigger.Request(), Timer.Request()
        self.drone_status_subscriber = self.create_subscription(Trials1, "status_update", self.drone_status_callback, 10)
        self.timer_control_client = self.create_client(Timer, 'timer_control')
        self.create_timer(0.1, self.turn_complete_callback)
        self.yaw_comparator, self.drone_turned, self.timer_complete , self.current_yaw = 0.0 , True, False, 0.0
        self.mission_id = ""
    def manual_mode_callback(self, request, response):
        self.trigger_manual_mode = request.trigger
        self.mission_id = request.mission_id
        if self.trigger_manual_mode:
            self.get_logger().info('I am in Manual mode')
            self.timer_request.trigger = True
            self.timer_request.time = 120.0
            self.timer_control_client.call_async(self.timer_request)
        else:
            self.get_logger().info('Manual mode deactivated')
            self.tracker_done.trigger = True
            self.tracking_done_client.call_async(self.tracker_done)
            self.timer_request.trigger = False
            self.timer_control_client.call_async(self.timer_request)

        response.success = True
        return response
    
    def turn_right_callback(self, request, response):
        self.get_logger().info("turning right")
        self.timer_request.trigger = False
        self.timer_control_client.call_async(self.timer_request)
        self.mission_update_request.mission_status = 3.0  #sends signal to say mission in progress, manual mode exited
        self.mission_update_request.mission_id = self.mission_id
        self.mission_update_client.call_async(self.mission_update_request)
        self.drone_request.message = "Turn"
        self.yaw_comparator = self.current_yaw + 30.0 - 5
        self.drone_turned = False
        self.drone_request.turn_yaw = self.current_yaw + 30.0
        self.turn_client.call_async(self.drone_request)
        response.success = True
        return response
        

    def turn_left_callback(self, request, response):
        self.get_logger().info("turning left")
        self.timer_request.trigger = False
        self.timer_control_client.call_async(self.timer_request)
        self.mission_update_request.mission_status = 3.0  #sends signal to say mission in progress, manual mode exited
        self.mission_update_request.mission_id = self.mission_id
        self.mission_update_client.call_async(self.mission_update_request)
        self.drone_request.message = "Turn"
        self.drone_turned = False
        self.yaw_comparator = self.current_yaw - 30.0 + 5
        self.drone_request.turn_yaw = self.current_yaw - 30.0
        self.turn_client.call_async(self.drone_request)
        response.success = True
        return response

    def drone_status_callback(self, msg):
        self.current_yaw = msg.current_yaw

    def turn_complete_callback(self):
        if self.current_yaw <= self.yaw_comparator and self.drone_turned == False:
            self.get_logger().info('turn complete')
            self.drone_turned = True
            self.tracker_done.trigger = True
            self.turn_complete_client.call_async(self.tracker_done)

    def timer_expired_callback(self, request, response):
        self.timer_complete = request.trigger
        self.get_logger().info(f'Manual mode aborted')
        self.tracker_done.trigger = True
        self.tracking_done_client.call_async(self.tracker_done)
        response.success = True
        return response




def main(args=None):
    rclpy.init(args=args)
    node = ManualMode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
