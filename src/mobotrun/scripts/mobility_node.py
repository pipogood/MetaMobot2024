#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from MobiControl import Mobility, Lidar
import sys, os, yaml
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool
import json
from rclpy import qos
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan

class MobilityNode(Node):
    def __init__(self):
        print("node int mobility")
        super().__init__('Mobility_node')
        self.MobiFunction = Mobility()
        self.LidarFunction = Lidar()
        Streaming = QoSProfile(durability=qos.QoSDurabilityPolicy.VOLATILE,
                           reliability=qos.QoSReliabilityPolicy.BEST_EFFORT, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        template = {"beware": 0, "stop": 0, "result": "ok"}
        self.Lidar1_status = dict.fromkeys(['Q3', 'Q4', 'Q5', 'Q6'], template)
        self.Lidar2_status = dict.fromkeys(['Q7', 'Q8', 'Q1', 'Q2'], template)

        self.current_vel = []
        self.status = ""
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.dxlvel_publisher = self.create_publisher(Float32MultiArray,'mobility/feedback/velocity',Streaming)
        self.status_publisher = self.create_publisher(String,'mobility/feedback/temp',Streaming)
        self.lidar_publisher = self.create_publisher(String,'lidars/status',Streaming)

        self.velo_subscription = self.create_subscription(String,'mobility/input/velocity',self.Drivedxl,Streaming)
        self.stop_subscription = self.create_subscription(String,'mobility/stop',self.Stopdxl,Streaming)

        #Lidars
        self.lidar1_subscription = self.create_subscription(LaserScan,'/lidar0/scan',self.lidar1 ,Streaming)
        self.lidar2_subscription = self.create_subscription(LaserScan,'/lidar1/scan',self.lidar2 ,Streaming)
      
    def timer_callback(self):
        self.current_vel = self.MobiFunction.GetVelocity() 
        self.status = self.MobiFunction.GetTempAndLoad()

        send_velocity = Float32MultiArray()
        send_status = String()
        send_lidar = String()

        send_velocity.data = [self.current_vel[0][0], self.current_vel[1][0], self.current_vel[2][0]]
        send_status.data = self.status

        send_lidar.data = str({
            'Q1': self.Lidar2_status['Q1']['result'], 
            'Q2': self.Lidar2_status['Q2']['result'],
            'Q3': self.Lidar1_status['Q3']['result'],
            'Q4': self.Lidar1_status['Q4']['result'],
            'Q5': self.Lidar1_status['Q5']['result'], 
            'Q6': self.Lidar1_status['Q6']['result'],
            'Q7': self.Lidar2_status['Q7']['result'],
            'Q8': self.Lidar2_status['Q8']['result'],
            })

        # print(send_velocity)

        self.dxlvel_publisher.publish(send_velocity)
        self.status_publisher.publish(send_status)
        self.lidar_publisher.publish(send_lidar)


    def Drivedxl(self, msg:String):
        self.st = json.loads(msg.data)
        self.vx = self.st["mobi_vx"]
        self.vy = self.st["mobi_vy"]
        self.wz = self.st["mobi_w"]

        print("recieve vel: ", self.vx, " ", self.vy, " ", self.wz)

        

        self.MobiFunction.SetWheelVelocity(Vx = self.vx, Vy = self.vy, Wz= self.wz)

    def Stopdxl(self, msg:String):
        self.MobiFunction.SetWheelVelocity(Vx = 0.0, Vy = 0.0, Wz= 0.0)

    def lidar1(self, msg:LaserScan):
        self.Lidar1_status = self.LidarFunction.computeLidar1(msg) #Quartier 3,4,5,6

    def lidar2(self, msg:LaserScan):
        self.Lidar2_status = self.LidarFunction.computeLidar2(msg) #Quartier 7,8,1,2
        

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