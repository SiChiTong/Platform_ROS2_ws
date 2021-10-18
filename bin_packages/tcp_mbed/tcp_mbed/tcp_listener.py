import socket

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class TCP_Publisher(Node):
    def __init__(self, address, port):

        super().__init__('tcp_publisher')        
        self.publisher_result = self.create_publisher(String, 'result', 10)
        
        self.HOST = address
        self.PORT = port
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.HOST, self.PORT))
        except Exception as e:
            self.get_logger().fatal("Fail to connect! %s." % e.strerror)
            self.destroy_node()            
            return

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        
        data = self.client_socket.recv(1024)
        if(data.__sizeof__() != 0):                
            msg.data = data.decode()

        self.publisher_result.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    tcp_pub = TCP_Publisher('192.168.0.200',4000)

    rclpy.spin(tcp_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tcp_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

