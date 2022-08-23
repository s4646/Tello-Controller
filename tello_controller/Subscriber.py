import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from socket import *
from djitellopy import tello
from threading import Thread
import cv2

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
        
        print("Battery percentage:", self.me.get_battery())
        self.video_thread = Thread(target=self.video)

        if self.me.get_battery() < 10:
            raise RuntimeError("Tello rejected attemp to takeoff due to low Battery")
        
        self.me.takeoff()
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
        if land != 0:
            self.me.land()
            print("LAND")
        elif takeoff != 0:
            self.me.takeoff()
            print("TAKEOFF")
        else:
            self.me.send_rc_control(int(a), int(b), int(c), int(d))

    def video(self):
        while True:
            image = self.me.get_frame_read().frame
            cv2.imshow("results", image)
            cv2.waitKey(1)

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