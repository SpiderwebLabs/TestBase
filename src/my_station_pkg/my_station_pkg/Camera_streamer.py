
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class VideoViewerNode(Node):
    def __init__(self):
        super().__init__('Camera_streamer1')
        self.publisher_ = self.create_publisher(Image, 'camera_image1', 10)  # Create an Image publisher
        self.ip_address = 'http://192.168.101.252:8000'  
        self.video_url = f'{self.ip_address}/video_feed'
        self.bridge = CvBridge()
        self.print_counter = 0

    def run(self):
        cap = cv2.VideoCapture(self.video_url)
        if not cap.isOpened():
            self.get_logger().error("Error: Could not open video stream.")
            return

        while True:
            #print(f'reading frame1')
            ret, frame = cap.read()
            #print(f'received frame1')
            if not ret:
                break

            # Publish the frame to the topic
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            try:
                self.publisher_.publish(image_msg)
                self.print_counter += 1  # Increment the counter
                print(f'getting frame1 [{self.print_counter}]')
            except Exception as e:
                print(f'error is{e}')



def main(args=None):
    rclpy.init(args=args)
    video_viewer_node = VideoViewerNode()
    video_viewer_node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
