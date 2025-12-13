import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket, struct, pickle
import numpy as np

GO1_IP = "192.168.123.161"   # Go1 IP
PORT = 9999

class LidarRelay(Node):
    def __init__(self):
        super().__init__('lidar_tcp_relay')

        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)

        # Connect to Go1
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((GO1_IP, PORT))

        self.get_logger().info("Connected to Go1 LiDAR TCP server.")

        self.timer = self.create_timer(0.001, self.receive)

    def recvall(self, n):
        data = b''
        while len(data) < n:
            packet = self.sock.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data

    def receive(self):
        # Read size header
        raw_len = self.recvall(4)
        if not raw_len:
            return
        msg_len = struct.unpack(">I", raw_len)[0]

        payload = self.recvall(msg_len)
        data = pickle.loads(payload)

        # Convert to ROS2 LaserScan
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"

        msg.angle_min = data["angle_min"]
        msg.angle_max = data["angle_max"]
        msg.angle_increment = data["angle_increment"]
        msg.range_min = data["range_min"]
        msg.range_max = data["range_max"]
        msg.ranges = data["ranges"]

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
