import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from socket import *
import time

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)

        self._rate = self.create_rate(1) # 1 Hz
        self.serverSocket = socket(AF_INET, SOCK_DGRAM)
        self.serverSocket.bind(('', 8889))

        self.serverSocket.sendto('command'.encode(), ('192.168.10.1', 8889))
        print("Command sent")
        data, _ = self.serverSocket.recvfrom(128)
        if data.decode() != 'ok':
            raise RuntimeError('Tello rejected attempt to enter command mode')
        print(data.decode())

        self.serverSocket.sendto('battery?'.encode(), ('192.168.10.1', 8889))
        data, _ = self.serverSocket.recvfrom(128)
        print("Battery percentage: ", data.decode())
        if int(data.decode()) < 10:
            raise RuntimeError("Tello rejected attemp to takeoff due to low Battery")
        
        time.sleep(1)
        self.serverSocket.sendto('takeoff'.encode(), ('192.168.10.1', 8889))
        print("Takeoff sent")
        data, _ = self.serverSocket.recvfrom(128)
        if data.decode() != 'ok':
            raise RuntimeError('Tello rejected attempt to takeoff')
        print(data.decode())
        time.sleep(1)

    def listener_callback(self, msg:Joy):
        # print(msg.axes)
        # print(msg.buttons)
        factor = 60

        data = list(msg.axes)
        a = -data[0] * factor
        b = data[1] * factor
        c = data[3] * factor
        d = -data[2] * factor

        data = list(msg.buttons)
        land = data[10]
        takeoff = data[11]
        if land != 0:
            command = "land"
            print("LAND")
        elif takeoff != 0:
            command = "takeoff"
            print("TAKEOFF")
        else:
            command = "rc {} {} {} {}".format(a, b, c, d)
        
        self.serverSocket.sendto(command.encode('utf-8'), ('192.168.10.1', 8889))
        print("Sent: {}".format(command))
        
        if land !=0 or takeoff != 0:
            data, _ = self.serverSocket.recvfrom(128)
            print(data.decode())
        
        time.sleep(0.01)


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