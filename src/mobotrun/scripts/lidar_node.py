#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.qos import QoSProfile
import sys, os, yaml
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool
import math
import numpy as np
import json
from sensor_msgs.msg import LaserScan

class Lidar(Node):
    def __init__(self):
        print("LidarNode_int")
        super().__init__('Lidar_node')

        self.D1_x = 0.18
        self.D1_y = 0.22
        self.D2_x = 0.18
        self.D2_y = 0.175
        self.shortest_distance_L1 = 5.0
        self.shortest_distance_L2 = 5.0
        self.shortest_distance = 5.0
        self.x_detect = 0.0
        self.y_detect = 0.0
        self.degree_detect = 0.0
        self.lidar_detect = ''

        Streaming = QoSProfile(durability=qos.QoSDurabilityPolicy.VOLATILE,
                           reliability=qos.QoSReliabilityPolicy.BEST_EFFORT, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=1)
        
        self.timer = self.create_timer(0.1, self.timer_callback)

        #Lidars
        self.lidar_mobi_publisher = self.create_publisher(Float32MultiArray,'/lidar/mobility',Streaming)
        self.lidar1_subscription = self.create_subscription(LaserScan,'/lidar0/scan',self.lidar1 ,Streaming)
        self.lidar2_subscription = self.create_subscription(LaserScan,'/lidar1/scan',self.lidar2 ,Streaming)


    def timer_callback(self):
        if self.shortest_distance_L1 < self.shortest_distance_L2:
            self.shortest_distance = self.shortest_distance_L1
        else:
            self.shortest_distance = self.shortest_distance_L2
            
        print("Shortest distance from ",self.lidar_detect, ": ", self.shortest_distance, "at ", self.degree_detect, self.x_detect, self.y_detect)

        send_lidar = Float32MultiArray()
        send_lidar.data = [self.shortest_distance, self.x_detect , self.y_detect]
        self.lidar_mobi_publisher.publish(send_lidar)

    def lidar1(self, msg:LaserScan):
        self.shortest_distance_L1 = 5.0  
        data = msg
        count = int(math.floor(data.scan_time/data.time_increment))
        degree_new = 0.0

        for i in range (0,count):
            if data.ranges[i] == None:
                continue

            degree = (data.angle_min + data.angle_increment * i)*57.2958
            degree = np.round(degree,3)

            if(degree >= -180 and degree <= -90):
                degree_new = np.abs(degree) - 90
            elif(degree >= 0 and degree <= 180):
                degree_new = (180-degree) + 90
            else:
                degree_new =  np.abs(degree) + 270

            if (degree_new >= 180 and degree_new <= 315): #Igore degree which detect robot
                pass
            else:
                if math.isinf(data.ranges[i]) == False:
                    
                    lidar_x = (data.ranges[i])*math.sin(degree_new/57.2958) *-1  #note +Y axis of Lidar is -X axis in robot frame
                    lidar_y = (data.ranges[i])*math.cos(degree_new/57.2958) #note: +X axis of Lidar is +Y axis in robot frame

                    # print("Front_Lidar:  degree-", degree_new, "lidar_y->", lidar_y, "lidar_x->", lidar_x)
            
                    robot_x = lidar_x - self.D1_x
                    robot_y = lidar_y + self.D1_y

                    if (np.sqrt(robot_x**2 + robot_y**2) < self.shortest_distance_L1) and (np.sqrt(robot_x**2 + robot_y**2) < self.shortest_distance_L2):
                        self.shortest_distance_L1 = np.sqrt(robot_x**2 + robot_y**2)
                        self.x_detect = robot_x
                        self.y_detect = robot_y
                        self.degree_detect = degree_new
                        self.lidar_detect = "L1"

    def lidar2(self, msg:LaserScan):
        self.shortest_distance_L2 = 5.0
        data = msg
        count = int(math.floor(data.scan_time/data.time_increment))
        degree_new = 0.0

        for i in range (0,count):
            if data.ranges[i] == None:
                continue

            degree = (data.angle_min + data.angle_increment * i)*57.2958
            degree = np.round(degree,3)

            if(degree >= -180 and degree <= -90):
                degree_new = np.abs(degree) - 90
            elif(degree >= 0 and degree <= 180):
                degree_new = (180-degree) + 90
            else:
                degree_new =  np.abs(degree) + 270


            if (degree_new >= 0 and degree_new <= 100): #Igore degree which detect robot
                pass
            else:
                if math.isinf(data.ranges[i]) == False:
                    lidar_x = (data.ranges[i])*math.sin(degree_new/57.2958) * -1  #note +Y axis of Lidar is -X axis in robot frame
                    lidar_y = (data.ranges[i])*math.cos(degree_new/57.2958) #note: +X axis of Lidar is +Y axis in robot frame

                    # print("Back_Lidar:  degree-", degree_new, "lidar_y->", lidar_y, "lidar_x->", lidar_x)

                    robot_x = lidar_x + self.D2_x
                    robot_y = lidar_y - self.D2_y

                    if (np.sqrt(robot_x**2 + robot_y**2) < self.shortest_distance_L1) and (np.sqrt(robot_x**2 + robot_y**2) < self.shortest_distance_L2):
                        self.shortest_distance_L2 = np.sqrt(robot_x**2 + robot_y**2)
                        self.x_detect = robot_x
                        self.y_detect = robot_y
                        self.degree_detect = degree_new
                        self.lidar_detect = "L2"            

def main(args=None):
    rclpy.init(args=args)
    node_object = Lidar()
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