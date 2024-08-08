#!/usr/bin/python3

import rclpy
from rclpy import qos
from rclpy.qos import QoSProfile
from rclpy.node import Node
import sys, os, yaml
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool
import json
import paho.mqtt.client as mqtt
from open_manipulator_msgs.msg import KinematicsPose
from sensor_msgs.msg import JointState

class MqttNode(Node):
    def __init__(self):
        print("mqttnode_int")
        super().__init__('MQTT_node')
        Streaming = QoSProfile(durability=qos.QoSDurabilityPolicy.VOLATILE,
                           reliability=qos.QoSReliabilityPolicy.BEST_EFFORT, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('host', None),
                ('port', None)])

        self.send_range = 0.0
        
        #user to META MOBOT MQTT Topic
        self.mobility_vel_fromUser = "user/mobility/velocity"
        self.mobility_stop_fromUser = "user/mobility/stop"
        self.mobility_velo_toUser = "mobility/user/velocity"
        self.mobility_temp_toUser = "mobility/user/temp"
        self.mobility_load_toUser = "mobility/user/load"

        self.mani_command_fromUser = "user/mani/command"
        self.mani_pose_toUser = "mani/user/pose"
        self.mani_joint_toUser = "mani/user/joint"

        self.lidar_range_toUser = "mobot/user/lidar/range"
        self.MQTT_sub_topic = [(self.mobility_vel_fromUser,0),(self.mobility_stop_fromUser,0), (self.mani_command_fromUser,0)]
        
        #Subscription topics within METAMOBOT
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.mobility_publish_vel = self.create_publisher(String, 'mobility/input/velocity', Streaming)
        self.mobility_publish_stop = self.create_publisher(String, 'mobility/stop', Streaming)
        self.mobi_velo_subscription = self.create_subscription(Float32MultiArray,'mobility/feedback/velocity',self.mobi_velo,Streaming)
        self.mobi_status_subscription = self.create_subscription(Float32MultiArray,'mobility/feedback/status',self.mobi_status,Streaming)

        self.mobi_status_subscription = self.create_subscription(Float32MultiArray,'lidars/status',self.lidar_status ,Streaming)

        self.mani_command_publish = self.create_publisher(String, 'mani/command', Streaming)
        self.mani_pose_subscription = self.create_subscription(Float32MultiArray,'mani/feedback/pose',self.mani_pose,Streaming)
        self.mani_joint_subscription = self.create_subscription(Float32MultiArray,'mani/feedback/joint',self.mani_joint,Streaming)

        self.client = mqtt.Client()
        self.client.on_message = self.main
        self.host = "broker.hivemq.com"
        self.port = 1883
        self.client.connect(self.host, self.port)
        self.client.on_connect = self.on_connect

        self.client.loop_start()

    def timer_callback(self):
        self.client.publish(self.lidar_range_toUser, json.dumps(self.send_range))

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
        self.client.subscribe(self.MQTT_sub_topic)           

    def main(self, client, userdata, msg):
        send = String()
        topic = msg.topic
        message = msg.payload.decode("utf-8")
        send.data = message 
        print(send)

        if topic == self.mobility_vel_fromUser:
            self.mobility_publish_vel.publish(send)

        elif topic == self.mobility_stop_fromUser:
            self.mobility_publish_stop.publish(send)  

        elif topic == self.mani_command_fromUser:
            self.mani_command_publish.publish(send)

    def mobi_velo(self, msg:Float32MultiArray):
        send = {'mobi_vx': float(msg.data[0]), 'mobi_vy' : float(msg.data[1]), 'mobi_w': float(msg.data[2])}
        self.client.publish(self.mobility_velo_toUser, json.dumps(send,sort_keys=True))

    def mobi_status(self,msg:Float32MultiArray):
        send_temp = {'mobi_1': float(msg.data[0]), 'mobi_2' : float(msg.data[2]), 'mobi_3': float(msg.data[4]), 'mobi_4': float(msg.data[6])}
        send_load = {'mobi_1': float(msg.data[1]), 'mobi_2' : float(msg.data[3]), 'mobi_3': float(msg.data[5]), 'mobi_4': float(msg.data[7])}
        self.client.publish(self.mobility_temp_toUser, json.dumps(send_temp))
        self.client.publish(self.mobility_load_toUser, json.dumps(send_load))

    def lidar_status(self, msg:Float32MultiArray):
        self.send_range = {'lidar_range': float(msg.data[0])}

    def mani_pose(self, msg:Float32MultiArray):
        send = {'x': float(msg.data[0]), 'y' : float(msg.data[1]), 'z': float(msg.data[2]), 'yaw': float(msg.data[3]), 'pitch' : float(msg.data[4]), 'roll': float(msg.data[5])}
        self.client.publish(self.mani_pose_toUser, json.dumps(send,sort_keys=True))

    def mani_joint(self, msg:Float32MultiArray):
        send = {'J1': float(msg.data[0]), 'J2' : float(msg.data[1]), 'J3': float(msg.data[2]), 'J4': float(msg.data[3]), 'J5' : float(msg.data[4]), 'J6': float(msg.data[5])}
        self.client.publish(self.mani_joint_toUser, json.dumps(send,sort_keys=True))


def main(args=None):
    rclpy.init(args=args)
    node_object = MqttNode()
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