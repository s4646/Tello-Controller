import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from socket import *
from djitellopy import tello
import time

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            5)

        self._rate = self.create_rate(5) # 5 Hz
        
        self.me = tello.Tello()
        self.me.connect()

        if self.me.get_battery()< 10:
            print("Battery percentage:", self.me.get_battery())
            raise RuntimeError("Tello rejected attemp to takeoff due to low Battery")
        
        self.me.takeoff()

    def listener_callback(self, msg:Joy):
        # print(msg.axes)
        # print(msg.buttons)
        big_factor = 100
        medium_factor = 50
        small_factor = 20

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