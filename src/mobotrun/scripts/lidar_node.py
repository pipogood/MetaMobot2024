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
        self.D1_y = 0.23
        self.D2_x = 0.14
        self.D2_y = 0.23        
        self.detect_degree = 60.0
        
        self.shortest_distance = 5.0
        self.shortest_distance_L1 = 5.0
        self.shortest_distance_L2 = 5.0
        self.x_detect = 0.0
        self.y_detect = 0.0
        self.degree_detect = 0.0
        self.mobi_direct = (0,0)
        self.lidar_detect = ''


        self.send_lidar = Float32MultiArray()

        Streaming = QoSProfile(durability=qos.QoSDurabilityPolicy.VOLATILE,
                           reliability=qos.QoSReliabilityPolicy.BEST_EFFORT, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=1)
        
        self.timer = self.create_timer(0.05, self.main)

        self.lidar_mobi_publisher = self.create_publisher(Float32MultiArray,'/lidar/mobility',Streaming)
        self.lidar_status_publisher = self.create_publisher(Float32MultiArray,'/lidars/status',Streaming)

        self.lidar1_subscription = self.create_subscription(LaserScan,'/lidar0/scan',self.lidar1 ,Streaming)
        self.lidar2_subscription = self.create_subscription(LaserScan,'/lidar1/scan',self.lidar2 ,Streaming)
        self.from_mobi_subscription = self.create_subscription(Float32MultiArray,'mobility/lidars',self.recieve_mobidirect ,Streaming)

       # New publisher for transformed LaserScan data
        self.transformed_lidar1_publisher = self.create_publisher(LaserScan, '/lidar0/transformed_scan', Streaming)
        self.transformed_lidar2_publisher = self.create_publisher(LaserScan, '/lidar1/transformed_scan', Streaming)



    def main(self):
        if self.shortest_distance_L1 < self.shortest_distance_L2:
            self.shortest_distance = self.shortest_distance_L1
        else:
            self.shortest_distance = self.shortest_distance_L2
            
        # print("Shortest distance from ",self.mobi_direct, ": ",self.lidar_detect, ": ", self.shortest_distance, "at ", self.degree_detect, self.x_detect, self.y_detect)

        self.send_lidar.data = [self.shortest_distance, self.x_detect , self.y_detect]
        self.lidar_mobi_publisher.publish(self.send_lidar)
        self.lidar_status_publisher.publish(self.send_lidar)

    def recieve_mobidirect(self, msg:Float32MultiArray):
        self.mobi_direct = (msg.data[0], msg.data[1])

    def lidar1(self, msg:LaserScan):
        self.shortest_distance_L1 = 5.0  
        data = msg
        transformed_ranges = [float('inf')] * len(data.ranges)
        count = int(math.floor(data.scan_time/data.time_increment))
        degree_new = 0.0

        for i in range (0,count):
            if data.ranges[i] == None:
                continue

            self.global_msg = msg

            degree = (data.angle_min + data.angle_increment * i)*57.2958
            degree = np.round(degree,3)

            if(degree >= -180 and degree <= -90):
                degree_new = np.abs(degree) - 90
            elif(degree >= 0 and degree <= 180):
                degree_new = (180-degree) + 90
            else:
                degree_new =  np.abs(degree) + 270

            if (degree_new >= 180 and degree_new <= 310): #Igore degree which detect robot
                pass
            else:
                if math.isinf(data.ranges[i]) == False:
                    
                    lidar_x = (data.ranges[i])*math.sin(degree_new/57.2958) *-1  #note +Y axis of Lidar is -X axis in robot frame
                    lidar_y = (data.ranges[i])*math.cos(degree_new/57.2958) #note: +X axis of Lidar is +Y axis in robot frame

                    # print("Front_Lidar:  degree-", degree_new, "lidar_y->", lidar_y, "lidar_x->", lidar_x)
                else:
                    lidar_x = 15
                    lidar_y = 15
            
                robot_x = lidar_x - self.D1_x
                robot_y = lidar_y + self.D1_y

                 ##########
                transformed_distance = np.sqrt(robot_x**2 + robot_y**2)
                if transformed_distance < 0.85:
                    transformed_ranges[i] = data.ranges[i]
                else:
                    transformed_ranges[i] = 0.0
                #########


                lidar1_detect_direct = (robot_x, robot_y) 
                dot_product = np.dot(self.mobi_direct, lidar1_detect_direct)
                mag_v1 = np.linalg.norm(self.mobi_direct)
                mag_v2 = np.linalg.norm(lidar1_detect_direct)
                cos_theta = dot_product / (mag_v1 * mag_v2)

                if np.degrees(np.arccos(cos_theta)) < self.detect_degree:
                    if (np.sqrt(robot_x**2 + robot_y**2) < self.shortest_distance_L1) and (np.sqrt(robot_x**2 + robot_y**2) < self.shortest_distance_L2):
                        self.shortest_distance_L1 = np.sqrt(robot_x**2 + robot_y**2)
                        self.x_detect = robot_x
                        self.y_detect = robot_y
                        self.degree_detect = degree_new
                        self.lidar_detect = "L1"

        self.publish_laser_scan1(msg, transformed_ranges)
   

    def lidar2(self, msg:LaserScan):
        self.shortest_distance_L2 = 5.0
        data = msg
        transformed_ranges = [float('inf')] * len(data.ranges)
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

            if (degree_new >= 0 and degree_new <= 120): #Igore degree which detect robot
                pass
            elif (degree_new >= 345 and degree_new <= 360):
                pass
            else:
                if math.isinf(data.ranges[i]) == False:
                    lidar_x = (data.ranges[i])*math.sin(degree_new/57.2958) * -1  #note +Y axis of Lidar is -X axis in robot frame
                    lidar_y = (data.ranges[i])*math.cos(degree_new/57.2958) #note: +X axis of Lidar is +Y axis in robot frame
                else:
                    lidar_x = 15
                    lidar_y = 15

                    # print("Back_Lidar:  degree-", degree_new, "lidar_y->", lidar_y, "lidar_x->", lidar_x)

                robot_x = lidar_x + self.D2_x
                robot_y = lidar_y - self.D2_y

                ########
                transformed_distance = np.sqrt(robot_x**2 + robot_y**2)
                if transformed_distance < 0.85:
                    transformed_ranges[i] = data.ranges[i]
                else:
                    transformed_ranges[i] = 0.0
                ########

                lidar2_detect_direct = (robot_x, robot_y) 
                dot_product = np.dot(self.mobi_direct, lidar2_detect_direct)
                mag_v1 = np.linalg.norm(self.mobi_direct)
                mag_v2 = np.linalg.norm(lidar2_detect_direct)
                cos_theta = dot_product / (mag_v1 * mag_v2)

                if np.degrees(np.arccos(cos_theta)) < self.detect_degree:
                    if (np.sqrt(robot_x**2 + robot_y**2) < self.shortest_distance_L1) and (np.sqrt(robot_x**2 + robot_y**2) < self.shortest_distance_L2):
                        self.shortest_distance_L2 = np.sqrt(robot_x**2 + robot_y**2)
                        self.x_detect = robot_x
                        self.y_detect = robot_y
                        self.degree_detect = degree_new

        self.publish_laser_scan2(msg, transformed_ranges)           

    def publish_laser_scan1(self, msg, transformed_ranges):
        """ Publish transformed LaserScan data """
        transformed_scan = LaserScan()
        transformed_scan.header = msg.header
        transformed_scan.angle_min = msg.angle_min
        transformed_scan.angle_max = msg.angle_max
        transformed_scan.angle_increment = msg.angle_increment
        transformed_scan.time_increment = msg.time_increment
        transformed_scan.scan_time = msg.scan_time
        transformed_scan.range_min = msg.range_min
        transformed_scan.range_max = msg.range_max
        transformed_scan.ranges = transformed_ranges
        transformed_scan.intensities = msg.intensities
        self.transformed_lidar1_publisher.publish(transformed_scan)


    def publish_laser_scan2(self, msg, transformed_ranges):
        """ Publish transformed LaserScan data """
        transformed_scan = LaserScan()
        transformed_scan.header = msg.header
        transformed_scan.angle_min = msg.angle_min
        transformed_scan.angle_max = msg.angle_max
        transformed_scan.angle_increment = msg.angle_increment
        transformed_scan.time_increment = msg.time_increment
        transformed_scan.scan_time = msg.scan_time
        transformed_scan.range_min = msg.range_min
        transformed_scan.range_max = msg.range_max
        transformed_scan.ranges = transformed_ranges
        transformed_scan.intensities = msg.intensities
        self.transformed_lidar2_publisher.publish(transformed_scan)


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