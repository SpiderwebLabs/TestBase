import rclpy
from rclpy.node import Node
from more_interfaces.srv import Timer, Comms, Trigger, Error
from more_interfaces.msg import Trials1


class AutoNode(Node):
    def __init__(self):
        super().__init__('auto_node')
        self.trigger_server = self.create_service(Trigger, 'auto_mode', self.automode_callback)
        self.tracking_done_client = self.create_client(Trigger, 'tracker_done')
        self.turn_client = self.create_client(Comms, 'turn')
        self.timer_control_client = self.create_client(Timer, 'timer_control')
        self.error_client = self.create_client(Error, 'error_tracking')
        self.timer_expired_server = self.create_service(Timer, 'timer_complete', self.timer_expired_callback)
        self.status_subscriber = self.create_subscription(Trials1, 'status_update', self.status_callback, 10)
        #self.drone_status_subscriber = self.create_subscription(Trials1, "status_update", self.drone_status_callback, 10)
        self.automode_start, self.timer_request, self.turn_request, self.timer_complete, self.tracker_done = None, Timer.Request(), Comms.Request(), False, Trigger.Request()
        self.stop_turning, self.current_yaw, self.error_request, self.turn_request, self.drone_turned, self.desired_yaw = False, 0.0, Error.Request(), Comms.Request(), False, 0.0


    def automode_callback(self, request, response):
        self.automode_start = request.trigger
        if self.automode_start:
            self.get_logger().info("Auto mode activated")
            self.timer_request.trigger = True
            self.timer_request.time = 20.0
            #Start timer node request
            self.timer_control_client.call_async(self.timer_request)
            self.stop_turnig = False
            self.timer_complete = False
            self.drone_turning()
        else:
            #Stop and reset timer node aswell as stop turning drone
            self.get_logger().info("Auto mode deactivated")
            self.stop_turnig = True
            self.timer_complete = True
            self.timer_request.trigger = False
            self.timer_control_client.call_async(self.timer_request)

        response.success = True
        return response
    
    def drone_turning(self):
        if not self.timer_complete and not self.stop_turning:
            self.get_logger().info('Drone turning in auto mode')
            self.error_request.current_yaw = self.current_yaw
            future = self.error_client.call_async(self.error_request)
            self.error_client.call_async(self.error_request).add_done_callback(self.yaw_request_callback)  # Call the service asynchronously
            self.turn_request.message = "Turn"
            if self.desired_yaw > 0:
                self.yaw_comparator = self.desired_yaw - 5
                self.right_auto_turn = True
            elif self.desired_yaw < 0:
                self.yaw_comparator = self.desired_yaw + 5
                self.left_auto_turn = True
            self.drone_turned = False
            self.turn_request.turn_yaw = self.desired_yaw
            self.turn_client.call_async(self.turn_request)

    def yaw_request_callback(self, future):
        response = future.result()
        self.desired_yaw = response.desired_yaw
        self.get_logger().info(f'Desired yaw: {self.desired_yaw}')


    def status_callback(self, msg):
        self.current_yaw = msg.current_yaw

    def timer_expired_callback(self, request, response):
        self.timer_complete = request.trigger
        self.get_logger().info(f'Auto mode aborted')
        self.tracker_done.trigger = True
        self.tracking_done_client.call_async(self.tracker_done)
        response.success = True
        return response





def main(args=None):
    rclpy.init(args=args)
    node = AutoNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
