#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.qos import QoSProfile
from MobiControl import Mobility
import sys, os, yaml
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool
import math
import numpy as np
import json
from open_manipulator_msgs.msg import KinematicsPose
from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import *

class ManipulatorNode(Node):
    def __init__(self):
        print("ManipulatorNode_int")
        super().__init__('Manipulator_node')
        self.send_pose = Float32MultiArray()
        self.send_joint = Float32MultiArray()
        self.send_pose.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.send_joint.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        Streaming = QoSProfile(durability=qos.QoSDurabilityPolicy.VOLATILE,
                           reliability=qos.QoSReliabilityPolicy.BEST_EFFORT, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=1)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.mani_command_subscription = self.create_subscription(String,'mani/command',self.mani_control,Streaming)

        self.mani_publish_pose = self.create_publisher(Float32MultiArray, 'mani/feedback/pose', Streaming)
        self.mani_publish_joint = self.create_publisher(Float32MultiArray, 'mani/feedback/joint', Streaming)

        self.mani_pose_subscription = self.create_subscription(KinematicsPose,'/kinematics_pose',self.mani_pose,Streaming)
        self.mani_joint_subscription = self.create_subscription(JointState,'/joint_states',self.mani_joint,Streaming)

    def timer_callback(self):
        self.mani_publish_pose.publish(self.send_pose)
        self.mani_publish_joint.publish(self.send_joint)

    def mani_control(self, msg:String):
        self.st = json.loads(msg.data)
        self.mode = self.st["mode"]
        if self.mode == "joint":
            pass
        else:
            pass


    def mani_pose(self, msg:KinematicsPose):
        yaw, pitch, roll = self.cal_YPR(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.send_pose.data = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, yaw, pitch, roll]
        # print("pose: ", send)

    def mani_joint(self, msg:JointState):
        self.send_joint.data = [msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5]]
        # print("joint: ",send)


    def cal_YPR(self, x, y, z, w):
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        pitch = math.asin(2.0 * (w * y - z * x))
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        
        return yaw, pitch, roll
        

def main(args=None):
    rclpy.init(args=args)
    node_object = ManipulatorNode()
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