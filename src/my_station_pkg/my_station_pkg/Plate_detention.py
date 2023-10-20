import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import os
import sys
#from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
from more_interfaces.msg import Carplate
sys.path.append('/home/spiderweb/Drone_ws/src/my_station_pkg/my_station_pkg')
from sort.sort import Sort

from my_station_pkg.util import get_car, read_license_plate, write_csv
from urllib.parse import quote



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

class VehicleTrackingNode(Node):
    def __init__(self):
        super().__init__('Plate_detention')
        self.get_logger().info('Initializing Vehicle Tracking Node')


        # Load models and video
        self.coco_model = YOLO('yolov8n.pt')
        #model path
        relative_path = '~/Licence_Plate_Models/runs/detect/train6/weights/best.pt'
        # Expand the tilde character to get the absolute path
        absolute_path = os.path.expanduser(relative_path)
        # Now you can use the absolute_path in your YOLO constructor
        self.license_plate_detector = YOLO(absolute_path)
        self.license_plate_detector = YOLO('/home/spiderweb/Downloads/runs(2)/runs/detect/train/weights/best.pt')
        #self.ip_address = 'http://192.168.101.225:8000'
        self.video_url = camera_url
        #self.video_url = f'{self.ip_address}/video_feed'

        #self.cap = cv2.VideoCapture('/home/spiderweb/Licence_Plate_Models/Highway.mp4')
        self.cap = cv2.VideoCapture(self.video_url)
        #self.cap = cv2.VideoCapture(self.video_url, cv2.CAP_FFMPEG)

        # Initialize SORT tracker
        self.mot_tracker = Sort()
        self.results = {}

        # Define the list of vehicles
        self.vehicles = [2, 3, 5, 7]

        # Initialize frame number
        self.frame_nmr = -1
        self.ret = True

    def run_tracking(self):
        while rclpy.ok():
            self.frame_nmr += 1
            self.ret, frame = self.cap.read()
            #cv2.imshow('frame',frame)

            if not self.ret:
                break
            #ros2 image 
            #frame_msg = CvBridge().cv2_to_imgmsg(frame, encoding='bgr8')

            # Resize the frame to 640x640
            frame = cv2.resize(frame, (640, 640))

            # Detect vehicles
            detections = self.coco_model(frame)[0]

            detections_ = []
            for detection in detections.boxes.data.tolist():
                x1, y1, x2, y2, score, class_id = detection
                if int(class_id) in self.vehicles:

                    detections_.append([x1, y1, x2, y2, score])

            # Track vehicles only if there are detections
            if len(detections_) > 0:
                track_ids = self.mot_tracker.update(np.asarray(detections_))

                # Draw bounding boxes on detected cars
                for track_id in track_ids:
                    car_info = self.results.get(track_id, None)
                    if car_info:
                        car_bbox = car_info['car']['bbox']
                        x1, y1, x2, y2 = map(int, car_bbox)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)


                # Detect license plates only if there are tracked vehicles
                if len(track_ids) > 0:
                    license_plates = self.license_plate_detector(frame)[0]
                    for license_plate in license_plates.boxes.data.tolist():
                        x1, y1, x2, y2, score, class_id = license_plate

                        # Assign license plate to car
                        xcar1, ycar1, xcar2, ycar2, car_id = get_car(license_plate, track_ids)

                        if car_id != -1:
                            # Crop license plate
                            license_plate_crop = frame[int(y1):int(y2), int(x1): int(x2), :]

                            # Process license plate
                            license_plate_crop_gray = cv2.cvtColor(license_plate_crop, cv2.COLOR_BGR2GRAY)
                            _, license_plate_crop_thresh = cv2.threshold(license_plate_crop_gray, 64, 255, cv2.THRESH_BINARY_INV)

                            # Read license plate number
                            license_plate_text, license_plate_text_score = read_license_plate(license_plate_crop_thresh)

                            if license_plate_text is not None:
                                self.results[car_id] = {'car': {'bbox': [xcar1, ycar1, xcar2, ycar2]},
                                                'license_plate': {'bbox': [x1, y1, x2, y2],
                                                                    'text': license_plate_text,
                                                                    'bbox_score': score,
                                                                    'text_score': license_plate_text_score}}
                                
                                # Draw bounding box around detected license plate
                                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                                # Put license plate text on the frame
                                cv2.putText(frame, license_plate_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Display the frame with bounding boxes
        cv2.imshow('frame', frame)
        cv2.waitKey(1)         
        write_csv(self.results, '/home/spiderweb/Drone_ws/src/my_station_pkg/my_station_pkg/test.csv')
                        

    def start_tracking(self):
        self.run_tracking()


def main(args=None):
    rclpy.init(args=args)
    vehicle_tracking_node = VehicleTrackingNode()
    vehicle_tracking_node.start_tracking()
    rclpy.spin(vehicle_tracking_node)
    vehicle_tracking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
