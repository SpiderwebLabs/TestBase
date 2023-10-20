import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import asyncio
import pika
from urllib.parse import quote
import time

username = "admin"
password = "Bewredips@2023"  # Your password with an '@'
ip_address = "192.168.101.201"
port = "554"
stream_path = "stream_path"  # Replace with your specific stream path

# URL encode the username and password
username = quote(username)
password = quote(password)

# Construct the URL
camera_url = f"rtsp://{username}:{password}@{ip_address}:{port}/cam/realmonitor?channel=1&subtype=1"


class VideoViewerNode(Node):
    def __init__(self):
        super().__init__('Camera_streamer1')
        self.publisher_ = self.create_publisher(Image, 'camera_image1', 10)
        #self.ip_address = 'http://192.168.101.252:8000'
        self.video_url = camera_url
        self.bridge = CvBridge()
        #cv2.namedWindow('Video Stream', cv2.WINDOW_NORMAL)
        self.print_counter = 0
        self.connection = None
        while self.connection is None:
            try:
                self.connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
            except Exception as e:
                print(f'error is {e}')
                print("Waiting for RabbitMQ")
                time.sleep(4)


        print("Connected to RabbitMQ")
        self.channel = self.connection.channel()
        self.channel.queue_declare(queue='camera_frames')
        print("Declared queue 'camera_frames'")
    async def run(self):
        try:
            cap = cv2.VideoCapture(self.video_url, cv2.CAP_FFMPEG)
        except Exception as e:
            print(f'error is {e}')
            return
        if not cap.isOpened():
            self.get_logger().error("Error: Could not open video stream.")
            return

        while True:
            #await asyncio.sleep(1)
            ret, frame = cap.read()
            if not ret:
                break
            _, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            try:
                self.publisher_.publish(image_msg)
                self.channel.basic_publish(exchange='',
                              routing_key='camera_frames',
                              body=frame_bytes)
                print("Published frame")
                self.print_counter += 1
                print(f'publishing frame1 [{self.print_counter}]')
                #await asyncio.sleep(0.1)
                #cv2.imshow('Video Stream', frame)
                # if cv2.waitKey(1) & 0xFF == ord('q'):
                #     break

            except Exception as e:
                print(f'error is {e}')

def main(args=None):
    rclpy.init(args=args)
    video_viewer_node = VideoViewerNode()
    # Wrap the coroutine run() in an asyncio event loop
    loop = asyncio.get_event_loop()
    loop.run_until_complete(video_viewer_node.run())
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()