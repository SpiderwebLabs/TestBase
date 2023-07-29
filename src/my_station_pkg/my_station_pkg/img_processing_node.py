
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg._compressed_image import CompressedImage
from cv_bridge import CvBridge
import flask
import cv2
from flask import Response
import sys
from rclpy.executors import ExternalShutdownException


frame=None
lock =threading.Lock()
app = flask.Flask(__name__)


@app.route('/video_feed')
def img_display():
    return Response(gen(),
                    mimetype = 'multipart/x-mixed-replace; boundary=frame')
def gen():
    while True:
        global frame
        if frame is not None:
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame.tobytes() +b'\r\n\r\n')
        
@app.route('/test')
def tester():
    return "hello"

class VideoServer(Node):
    def __init__(self):
        super().__init__('video_server')
        self.subscription = self.create_subscription(
            CompressedImage, 'video_data', self.image_callback, 100 )
        self.cv_bridge = CvBridge()
        self.image = None

    def image_callback(self, msg):
        global frame
        with lock:
            frame = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "passthrough")
            _,frame = cv2.imencode('.jpg',frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
        
        
def run_flask_server():
    app.run(host = '0.0.0.0', port = 8080, threaded = True)

def main(args = None):
    rclpy.init(args = args)
    video_server = VideoServer()
    flask_thread = threading.Thread(target = run_flask_server)
    flask_thread.start()
    rclpy.spin(video_server)
    video_server.destroy_node()
    rclpy.shutdown()
  
 
if __name__ == '__main__':
    main()