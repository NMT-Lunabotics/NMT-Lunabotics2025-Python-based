#!/usr/bin/env python3
import threading
import time
import cv2
import rclpy
import numpy as np

from flask import Flask, Response
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

web_app = Flask(__name__)
latest_frame = None
frame_lock = threading.Lock()


class CameraWebStreamer(Node):
    def __init__(self):
        super().__init__('webhost_node')
        self.create_subscription(CompressedImage,'/camera/stream',self.image_callback,10)

    def image_callback(self, msg):
        global latest_frame
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None: return
        with frame_lock: latest_frame = frame

@web_app.route('/')
def index():
    return '''
    <html>
    <body style="margin:0;background:black;display:flex;justify-content:center;align-items:center;height:100vh;">
        <img src="/video_feed" style="width:auto;height:100vh;object-fit:contain;">
    </body>
    </html>
    '''

@web_app.route('/video_feed')
def video_feed():
    return Response(mjpeg_stream(),mimetype='multipart/x-mixed-replace; boundary=frame')

def mjpeg_stream():
    global latest_frame
    last_encoded = None
    while True:
        time.sleep(0.03) 
        with frame_lock: frame = latest_frame
        if frame is None: continue
        encode_success, buffer = cv2.imencode('.jpg',frame,[int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if not encode_success: continue
        jpeg = buffer.tobytes()
        if last_encoded == jpeg:continue
        last_encoded = jpeg
        yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' +jpeg +b'\r\n')

def start_web_server():
    web_app.run(host='0.0.0.0',port=8080,threaded=True,debug=False,use_reloader=False)

def main():
    rclpy.init()
    node = CameraWebStreamer()
    thread = threading.Thread(target=start_web_server, daemon=True)
    thread.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()