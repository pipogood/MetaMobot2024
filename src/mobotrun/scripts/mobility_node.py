#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from MobiControl import Mobility
import sys, os, yaml
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool
import json
from rclpy import qos
from rclpy.qos import QoSProfile

class MobilityNode(Node):
    def __init__(self):
        print("node int mobility")
        super().__init__('Mobility_node')
        self.MobiFunction = Mobility()
        Streaming = QoSProfile(durability=qos.QoSDurabilityPolicy.VOLATILE,
                           reliability=qos.QoSReliabilityPolicy.BEST_EFFORT, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.current_vel = []
        self.status = ""
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.dxlvel_publisher = self.create_publisher(Float32MultiArray,'mobility/feedback/velocity',Streaming)
        self.status_publisher = self.create_publisher(String,'mobility/feedback/temp',Streaming)
        self.velo_subscription = self.create_subscription(String,'mobility/input/velocity',self.Drivedxl,Streaming)
        self.stop_subscription = self.create_subscription(String,'mobility/stop',self.Stopdxl,Streaming)
        
    def timer_callback(self):
        self.current_vel = self.MobiFunction.GetVelocity() 
        self.status = self.MobiFunction.GetTempAndLoad()

        send_velocity = Float32MultiArray()
        send_status = String()


        send_velocity.data = [self.current_vel[0][0], self.current_vel[1][0], self.current_vel[2][0]]
        send_status.data = self.status

        # print(send_velocity)

        self.dxlvel_publisher.publish(send_velocity)
        self.status_publisher.publish(send_status)


    def Drivedxl(self, msg:String):
        self.st = json.loads(msg.data)
        self.vx = self.st["mobi_vx"]
        self.vy = self.st["mobi_vy"]
        self.wz = self.st["mobi_w"]

        print("recieve vel: ", self.vx, " ", self.vy, " ", self.wz)

        self.MobiFunction.SetWheelVelocity(Vx = self.vx, Vy = self.vy, Wz= self.wz)

    def Stopdxl(self, msg:String):
        self.MobiFunction.SetWheelVelocity(Vx = 0.0, Vy = 0.0, Wz= 0.0)
        

def main(args=None):
    rclpy.init(args=args)
    node_object = MobilityNode()
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