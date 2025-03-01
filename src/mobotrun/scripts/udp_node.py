#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.qos import QoSProfile
import socket
import subprocess
import threading
import os
import sys
from std_msgs.msg import String  # Replace with your custom message type if needed.

class UdpListenerNode(Node):
    def __init__(self):
        super().__init__('udp_listener')
        
        # Create a UDP socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        Streaming = QoSProfile(durability=qos.QoSDurabilityPolicy.VOLATILE,
                           reliability=qos.QoSReliabilityPolicy.BEST_EFFORT, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=1)
        
        # Get parameters
        self.udp_host = self.declare_parameter('udp_host', '0.0.0.0').value
        self.udp_port = self.declare_parameter('udp_port', 1669).value
        self.udp_socket.bind((self.udp_host, self.udp_port))
        
        self.get_logger().info(f"Listening for UDP data on {self.udp_host}:{self.udp_port}...")
        
        # Publisher
        self.mobility_publish_vel = self.create_publisher(String, 'mobility/input/velocity', Streaming)
        
        # Timer to call the receive function
        self.timer = self.create_timer(0.01, self.receive_udp_data)


        

    def receive_udp_data(self):
        try:
            # Non-blocking check for UDP data
            self.udp_socket.settimeout(0.01)
            data, addr = self.udp_socket.recvfrom(1024)
            message = data.decode('utf-8')
            self.get_logger().info(f"Received message: {message} from {addr}")
            self.mobility_publish_vel.publish(String(data=message))
        except socket.timeout:
            pass  # No data received during timeout period
        except Exception as e:
            self.get_logger().error(f"Error receiving UDP data: {e}")
    
    def destroy_node(self):
        self.udp_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node_object = UdpListenerNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node_object)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        node_object.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()
