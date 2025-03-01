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
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
import numpy as np

class MobilityNode(Node):
    def __init__(self):
        print("node int mobility")
        super().__init__('Mobility_node')
        Streaming = QoSProfile(durability=qos.QoSDurabilityPolicy.VOLATILE,
                           reliability=qos.QoSReliabilityPolicy.BEST_EFFORT, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=1)
        
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_message_time = time.time()

        #adjust parameters
        self.declare_parameter('dxl_port','/dev/ttyUSB3')
        self.declare_parameter('lidar', True)

        self.open_lidar = self.get_parameter('lidar').get_parameter_value().bool_value
        self.dxl_port = self.get_parameter('dxl_port').get_parameter_value().string_value
        self.speed = 30.0 #init mobot speed
        self.timeout = 1.0
        self.ACC = 30.0

        print("Is lidar open? ", self.open_lidar)
        self.MobiFunction = Mobility(port = self.dxl_port)

        #For Lidar
        self.Lidar_distance = 0.0
        self.lidar_slow_factor = 1.0
        self.Lidar_xsign = 1
        self.Lidar_ysign = 1
        self.Lidar_stop_distance = 0.65
        self.Lidar_warn_distance = 0.85     
        self.Lidar_status = "OK"

        self.current_vel = []
        self.mobi_status = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.send_velocity = Float32MultiArray()
        self.send_mobi_status = Float32MultiArray()
        self.send_to_lidar = Float32MultiArray()
        self.send_lidar_touser = Float32MultiArray()

        self.timer = self.create_timer(0.1, self.main)

        self.dxlvel_publisher = self.create_publisher(Float32MultiArray,'mobility/feedback/velocity',Streaming)
        self.status_publisher = self.create_publisher(Float32MultiArray,'mobility/feedback/status',Streaming)

        self.velo_subscription = self.create_subscription(String,'mobility/input/velocity',self.Drivedxl,Streaming)
        self.stop_subscription = self.create_subscription(String,'mobility/stop',self.Stopdxl,Streaming)

        #Lidars
        self.mobi_to_lidar_publisher = self.create_publisher(Float32MultiArray,'mobility/lidars',Streaming)
        self.lidar_subscription = self.create_subscription(Float32MultiArray,'/lidar/mobility',self.get_lidar ,Streaming)

        #Offline keyboard 
        self.subscription = self.create_subscription(Twist,'cmd_vel',self.twist_callback,Streaming)
      
    def main(self):
        current_time = time.time()

        if self.open_lidar:
            self.check_obstacle_direct()

        if current_time - self.last_message_time > self.timeout:
            self.MobiFunction.SetWheelVelocity(Vx = 0.0, Vy = 0.0, Wz= 0.0, ACC = 20)
            self.MobiFunction.disable_torque(Vx = self.vx, Vy = self.vy, Wz= self.wz)
        else:
            if self.vx == 0 and self.vy == 0 and self.wz == 0:
                self.ACC = 50
            else:
                self.ACC = 30

            self.MobiFunction.SetWheelVelocity(
                                            Vx  = self.vx*self.speed/200/self.lidar_slow_factor, 
                                            Vy  = self.vy*self.speed/200/self.lidar_slow_factor, 
                                            Wz  = self.wz*self.speed/30/self.lidar_slow_factor, 
                                            ACC = self.ACC
                                            )

            self.current_vel = self.MobiFunction.GetVelocity() 
            self.mobi_status = self.MobiFunction.GetTempAndLoad()
            self.send_velocity.data = [self.current_vel[0][0], self.current_vel[1][0], self.current_vel[2][0]]
            self.send_mobi_status.data = self.mobi_status
            self.dxlvel_publisher.publish(self.send_velocity)
            self.status_publisher.publish(self.send_mobi_status)


    def Drivedxl(self, msg:String):
        self.last_message_time = time.time()
        now = self.get_clock().now().to_msg()
        sec = now.sec
        nanosec = now.nanosec

        # Convert seconds to hours, minutes, and seconds
        hours = (sec // 3600) % 24
        minutes = (sec // 60) % 60
        seconds = sec % 60
        mili_sec = round(nanosec // 1000000,2)
        print(f'Current Time:{minutes:02d}:{seconds:02d}.{mili_sec:02d}')

        # print(f'Unix Epoch Time: {now}')

        self.st = json.loads(msg.data)
        if list(self.st['payload'].keys())[0] == 'cmd':
            self.speed = self.st['payload']['param']
        
        if list(self.st['payload'].keys())[0] == 'command':
            self.vx = self.st['payload']['value'][1]
            self.vy = self.st['payload']['value'][0]
            self.wz = self.st['payload']['value'][2] * -1

        # print("recieve vel: ", self.vx*self.speed / 200, " ", self.vy*self.speed / 200, " ", self.wz*self.speed / 20, self.speed)

    def Stopdxl(self, msg:String):
        self.MobiFunction.SetWheelVelocity(Vx = 0.0, Vy = 0.0, Wz= 0.0)

    def Shutdown(self):
        self.MobiFunction.Shutdown()

    def get_lidar(self, msg:Float32MultiArray):
        self.Lidar_distance = msg.data[0]
        self.Lidar_xsign = msg.data[1]
        self.Lidar_ysign = msg.data[2]

    def check_obstacle_direct(self):

        self.send_to_lidar.data = [float(self.vy), float(self.vx)]
        if (np.sign(self.vy), np.sign(self.vx)) != (0,0):
            self.mobi_to_lidar_publisher.publish(self.send_to_lidar) #to lidar node

        if self.Lidar_distance < self.Lidar_stop_distance:
            self.Lidar_status = "stop"
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0

        elif self.Lidar_distance < self.Lidar_warn_distance:
            self.Lidar_status = "beware"
            self.lidar_slow_factor = 3.0

        elif self.Lidar_distance > self.Lidar_warn_distance:
            self.Lidar_status = "OK"
            self.lidar_slow_factor = 1.0

        # print("Lidar_status: ", self.Lidar_status, self.Lidar_distance)

    def twist_callback(self, msg: Twist):
        self.last_message_time = time.time()

        self.vx = np.sign(msg.linear.x)
        self.wz = np.sign(msg.angular.z) * -1
        self.speed = 50

        # print("recieve vel: ", self.vx*self.speed / 200, " ", self.vy*self.speed / 200, " ", self.wz*self.speed / 20, self.speed)

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
        time.sleep(1)
        node_object.Shutdown()
        print("shutdown")
        time.sleep(1)
        node_object.destroy_node()
        rclpy.shutdown() 


if __name__ == '__main__':
    main()