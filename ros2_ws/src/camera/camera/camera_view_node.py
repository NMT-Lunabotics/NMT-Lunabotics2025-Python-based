#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import threading
import time
from flask import Flask, Response

class CameraViewNode(Node):
    def __init__(self):
        super().__init__('camera_view_node')

        self.use_gui=True
        self.webstream=True
        self.latest_frame=None
        self.lock=threading.Lock()

        self.last_display_time=0.0
        self.fps_limit=20.0

        self.image_topic='/camera/stream'
        self.window_name='Camera View'
        self.window_width=800
        self.window_height=600
        self.fullscreen=False
        self.web_port=8080

        self.create_subscription(CompressedImage,self.image_topic,self.image_callback,10)

        cv2.namedWindow(self.window_name,cv2.WINDOW_NORMAL)
        if self.fullscreen: cv2.setWindowProperty(self.window_name,cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
        else: cv2.resizeWindow(self.window_name,self.window_width,self.window_height)

        self.create_timer(0.03,self.update_display)

        self.web_app=Flask(__name__)
        self.web_app.add_url_rule('/', 'index', self.index)
        self.web_app.add_url_rule('/video', 'video', self.video)

        threading.Thread(target=self.run_web,daemon=True).start()

    def image_callback(self,msg):
        np_arr=np.frombuffer(msg.data,np.uint8)
        frame=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        with self.lock: self.latest_frame=frame

    def update_display(self):
        now=time.time()
        if now-self.last_display_time<1.0/self.fps_limit: return
        self.last_display_time=now

        with self.lock:
            if self.latest_frame is None: return
            display=self.latest_frame.copy()
        if not self.fullscreen: display=cv2.resize(display,(self.window_width,self.window_height),interpolation=cv2.INTER_LINEAR)

        cv2.imshow(self.window_name,display)
        cv2.waitKey(1)

    def index(self): return '<img src="/video" style="width:auto;height:100vh;object-fit:contain;">'

    def video(self): return Response(self.gen(),mimetype='multipart/x-mixed-replace; boundary=frame')

    def gen(self):
        last_time=0.0
        while True:
            time.sleep(0.03)
            now=time.time()
            if now-last_time<1.0/self.fps_limit: continue
            last_time=now
            with self.lock:
                if self.latest_frame is None: continue
                frame=self.latest_frame.copy()
            _,buf=cv2.imencode('.jpg',frame,[int(cv2.IMWRITE_JPEG_QUALITY),80])
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+buf.tobytes()+b'\r\n'

    def run_web(self): self.web_app.run(host='0.0.0.0',port=self.web_port,threaded=True,debug=False,use_reloader=False)

def main():
    rclpy.init()
    node=CameraViewNode()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()