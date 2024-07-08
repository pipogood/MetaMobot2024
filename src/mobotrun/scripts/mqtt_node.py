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

class MqttNode(Node):
    def __init__(self):
        print("mqttnode_int")
        super().__init__('MQTT_node')
        State = QoSProfile(durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=qos.QoSReliabilityPolicy.RELIABLE, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=20)
        Streaming = QoSProfile(durability=qos.QoSDurabilityPolicy.VOLATILE,
                           reliability=qos.QoSReliabilityPolicy.BEST_EFFORT, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('host', None),
                ('port', None)])
        
        #user to META MOBOT MQTT Topic
        self.mobility_vel_fromUser = "user/mobility/velocity"
        self.mobility_stop_fromUser = "user/mobility/stop"
        self.mobility_velo_toUser = "mobility/user/velocity"
        self.mobility_temp_toUser = "mobility/user/temp"
        self.MQTT_sub_topic = [(self.mobility_vel_fromUser,0),(self.mobility_stop_fromUser,0)]
        
        #Subscription topics within METAMOBOT
        self.mobility_publish_vel = self.create_publisher(String, 'mobility/input/velocity', Streaming)
        self.mobility_publish_stop = self.create_publisher(String, 'mobility/stop', Streaming)
        self.mobi_velo_subscription = self.create_subscription(Float32MultiArray,'mobility/feedback/velocity',self.mobi_velo,10)
        self.mobi_status_subscription = self.create_subscription(String,'mobility/feedback/temp',self.mobi_status,10)

        self.client = mqtt.Client()
        self.client.on_message = self.topic_get
        self.host = "broker.hivemq.com"
        self.port = 1883
        self.client.connect(self.host, self.port)
        self.client.on_connect = self.on_connect

        self.client.subscribe("mqtt/topic", qos=0) #to subscribe more than one topic add more subscribe lines
        self.client.loop_forever()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
        self.client.subscribe(self.MQTT_sub_topic)           

    def topic_get(self, client, userdata, msg):
        send = String()
        topic = msg.topic
        message = msg.payload.decode("utf-8")
        print(message)
        send.data = message 

        if topic == self.mobility_vel_fromUser:
            self.mobility_publish_vel.publish(send)

        elif topic == self.mobility_stop_fromUser:
            self.mobility_publish_stop.publish(send)   

    def mobi_velo(self, msg:Float32MultiArray):
        send = {'mobi_vx': float(msg.data[0]), 'mobi_vy' : float(msg.data[1])*-1, 'mobi_w': float(msg.data[2])}
        self.client.publish(self.mobility_velo_toUser, json.dumps(send,sort_keys=True))

    def mobi_status(self,msg:String):
        send = msg.data
        self.client.publish(self.mobility_temp_toUser, send)

        
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