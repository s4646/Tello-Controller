import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from socket import *
from djitellopy import tello
from threading import Thread
import cv2
import queue

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)

        self._rate = self.create_rate(10) # 10 Hz
        
        self.me = tello.Tello()
        self.me.connect()
        self.me.streamon()

        self.cap: cv2.VideoCapture = self.me.get_video_capture()
        self.q = queue.Queue()
        
        print("Battery percentage:", self.me.get_battery())
        self.video_thread = Thread(target=self.video)
        self.stream_thread = Thread(target=self.stream)

        if self.me.get_battery() < 10:
            raise RuntimeError("Tello rejected attemp to takeoff due to low Battery")
        
        # self.me.takeoff()
        self.faceCascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.stream_thread.start()
        self.video_thread.start()

    def listener_callback(self, msg:Joy):
        big_factor = 100
        medium_factor = 50

        data = list(msg.axes)
        a = -data[0] * big_factor    # Left / Right
        b = data[1] * big_factor     # Forward / Backward
        c = data[3] * medium_factor  # Up / Down
        d = -data[2] * big_factor    # Yaw

        data = list(msg.buttons)
        land = data[10]     # 11
        takeoff = data[11]  # 12
        emergency = data[6] # 7
        battery = data[8]   # 9
        
        if land != 0:
            self.me.land()
            print("LAND")
        elif takeoff != 0:
            self.me.takeoff()
            print("TAKEOFF")
        elif battery != 0:
            print("Battery percentage:", self.me.get_battery())
        elif emergency != 0:
            try:
                print("EMERGENCY")
                self.me.emergency()
            except Exception as e:
                print("Did not receive OK, reconnecting to Tello")
                self.me.connect()

        else:
            self.me.send_rc_control(int(a), int(b), int(c), int(d))

    def video(self):
        while True:
            try:
                image = self.q.get()
            except queue.Empty:
                continue
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            faces = self.faceCascade.detectMultiScale(
            gray,     
            scaleFactor=1.2,
            minNeighbors=5,     
            minSize=(20, 20)
            )
            for (x,y,w,h) in faces:
                cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)
            cv2.imshow("results", image)
            cv2.waitKey(1)
    
    def stream(self):
        while True:
            ret, frame = self.cap.read()
            if not self.q.empty():
                try:
                    self.q.get_nowait()
                except queue.Empty:
                    pass
            self.q.put(frame)

def main(args=None):


    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()