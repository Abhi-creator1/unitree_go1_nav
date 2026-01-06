#!/usr/bin/env python
import rospy
import socket
import pickle
import struct
from sensor_msgs.msg import LaserScan

LAPTOP_IP = "192.168.12.17"   # laptop IP
PORT = 9999

def send_scan(msg):
    data = {
        "stamp": msg.header.stamp.to_sec(),
        "angle_min": msg.angle_min,
        "angle_max": msg.angle_max,
        "angle_increment": msg.angle_increment,
        "range_min": msg.range_min,
        "range_max": msg.range_max,
        "ranges": msg.ranges,
    }

    payload = pickle.dumps(data, protocol=4)
    header = struct.pack(">I", len(payload))
    conn.sendall(header + payload)

rospy.init_node("lidar_forwarder")

# Create TCP server
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(("0.0.0.0", PORT))
server.listen(1)
print("Waiting for laptop to connect...")
conn, addr = server.accept()
print("Connected:", addr)

sub = rospy.Subscriber("/scan", LaserScan, send_scan, queue_size=1)
rospy.spin()
