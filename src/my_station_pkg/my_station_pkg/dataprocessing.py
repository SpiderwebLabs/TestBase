import rclpy
from rclpy.node import Node
import csv
import numpy as np
from scipy.interpolate import interp1d
from more_interfaces.msg import Carplate
from statistics import mode, StatisticsError  

class LicensePlateProcessingNode(Node):
    def __init__(self):
        super().__init__('dataprocessing')
        self.get_logger().info('Initializing License Plate Processing Node')
        # Initialize the ROS publisher for results (you might want to customize the message type)
        self.result_pub = self.create_publisher(Carplate, '/add_vehicle', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) 

    def interpolate_bounding_boxes(self, data):
        # Extract necessary data columns from input data
        frame_numbers = np.array([int(row['frame_nmr']) for row in data])
        car_ids = np.array([int(float(row['car_id'])) for row in data])
        car_bboxes = np.array([list(map(float, row['car_bbox'][1:-1].split())) for row in data])
        license_plate_bboxes = np.array([list(map(float, row['license_plate_bbox'][1:-1].split())) for row in data])

        interpolated_data = []
        unique_car_ids = np.unique(car_ids)
        for car_id in unique_car_ids:

            frame_numbers_ = [p['frame_nmr'] for p in data if int(float(p['car_id'])) == int(float(car_id))]
            print(frame_numbers_, car_id)

            # Filter data for a specific car ID
            car_mask = car_ids == car_id
            car_frame_numbers = frame_numbers[car_mask]
            car_bboxes_interpolated = []
            license_plate_bboxes_interpolated = []

            first_frame_number = car_frame_numbers[0]
            last_frame_number = car_frame_numbers[-1]

            for i in range(len(car_bboxes[car_mask])):
                frame_number = car_frame_numbers[i]
                car_bbox = car_bboxes[car_mask][i]
                license_plate_bbox = license_plate_bboxes[car_mask][i]

                if i > 0:
                    prev_frame_number = car_frame_numbers[i-1]
                    prev_car_bbox = car_bboxes_interpolated[-1]
                    prev_license_plate_bbox = license_plate_bboxes_interpolated[-1]

                    if frame_number - prev_frame_number > 1:
                        # Interpolate missing frames' bounding boxes
                        frames_gap = frame_number - prev_frame_number
                        x = np.array([prev_frame_number, frame_number])
                        x_new = np.linspace(prev_frame_number, frame_number, num=frames_gap, endpoint=False)
                        interp_func = interp1d(x, np.vstack((prev_car_bbox, car_bbox)), axis=0, kind='linear')
                        interpolated_car_bboxes = interp_func(x_new)
                        interp_func = interp1d(x, np.vstack((prev_license_plate_bbox, license_plate_bbox)), axis=0, kind='linear')
                        interpolated_license_plate_bboxes = interp_func(x_new)

                        car_bboxes_interpolated.extend(interpolated_car_bboxes[1:])
                        license_plate_bboxes_interpolated.extend(interpolated_license_plate_bboxes[1:])

                car_bboxes_interpolated.append(car_bbox)
                license_plate_bboxes_interpolated.append(license_plate_bbox)

            for i in range(len(car_bboxes_interpolated)):
                frame_number = first_frame_number + i
                row = {}
                row['frame_nmr'] = str(frame_number)
                row['car_id'] = str(car_id)
                row['car_bbox'] = ' '.join(map(str, car_bboxes_interpolated[i]))
                row['license_plate_bbox'] = ' '.join(map(str, license_plate_bboxes_interpolated[i]))

                if str(frame_number) not in frame_numbers_:
                    # Imputed row, set the following fields to '0'
                    row['license_plate_bbox_score'] = '0'
                    row['license_number'] = '0'
                    row['license_number_score'] = '0'
                else:
                    # Original row, retrieve values from the input data if available
                    original_row = [p for p in data if int(p['frame_nmr']) == frame_number and int(float(p['car_id'])) == int(float(car_id))][0]
                    row['license_plate_bbox_score'] = original_row['license_plate_bbox_score'] if 'license_plate_bbox_score' in original_row else '0'
                    row['license_number'] = original_row['license_number'] if 'license_number' in original_row else '0'
                    row['license_number_score'] = original_row['license_number_score'] if 'license_number_score' in original_row else '0'

                interpolated_data.append(row)
        return interpolated_data
        
      
    def calculate_mode(self, data):
        try:
            return mode(data)
        except StatisticsError:
            return None
        
    def vehicle_type(self, class_id) :
        class_mapping = {
                        2: "car",
                        3: "motorbike",
                        5: "bus",
                        7: "truck"
                    }
        return class_mapping.get(class_id, "Unknown class")

    def timer_callback(self):
        with open('/home/spiderweb/Drone_ws/src/my_station_pkg/my_station_pkg/test.csv', 'r') as file:
            reader = csv.DictReader(file)
            data = list(reader)

        interpolated_data = self.interpolate_bounding_boxes(data)
        print(interpolated_data)
        type_vehicle = [self.vehicle_type(int(item['car_id'])) for item in interpolated_data]


        # Publish results
        result_msg = Carplate()
       # result_msg.plate_info = interpolated_data['license_number']

        #result_msg.type_vehicle = type_vehicle
        #result_msg.camera_id = "1187161a-c8e4-4642-9fa8-b225d6c1d049"
        #self.publisher.publish(result_msg)
       # self.get_logger().info(f'Detection: {result_msg}')

        header = ['frame_nmr', 'car_id', 'car_bbox', 'license_plate_bbox', 'license_plate_bbox_score', 'license_number', 'license_number_score']
        with open('test_interpolated.csv', 'w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=header)
            writer.writeheader()
            writer.writerows(interpolated_data)

def main(args=None):
    rclpy.init(args=args)
    license_plate_processing_node = LicensePlateProcessingNode()
    rclpy.spin(license_plate_processing_node)
    license_plate_processing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
