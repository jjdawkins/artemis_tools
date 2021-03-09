#! /usr/bin/env python3

import rospy
import math
import numpy as np
import socket
import json
from rospy_message_converter import json_message_converter,  message_converter

from std_msgs.msg import Empty, String, Header
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped
#from geographic_msgs.msg import GeoPoseStamped
from nav_msgs.msg import Odometry


class stateSender:
    def __init__(self):

        self.udp_ip = rospy.get_param('swat/udp_ip', '192.168.8.3')
        self.udp_port = rospy.get_param('swat/udp_port',9090)
        self.send_dt = rospy.get_param('swat/send_period',0.1)
        self.adverise_dt = rospy.get_param('swat/adverise_period',10)
        self.mocap_sub = rospy.Subscriber('/qualisys/Truck1/odom', Odometry,self.odomCallBack)
        self.pose_sub = rospy.Subscriber('/qualisys/Truck1/pose', PoseStamped,self.poseCallBack)
        self.advertise_timer = rospy.Timer(rospy.Duration(self.adverise_dt),self.advertiseCallBack)
        #self.send_timer = rospy.Timer(rospy.Duration(self.send_dt), self.sendCallBack)

        self.sock = socket.socket(socket.AF_INET, # Internet
                                 socket.SOCK_DGRAM) # UDP
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.odom_msg = Odometry()
        self.pose_msg = PoseStamped()
#Start every message with MEssage followed my a unique identifier, latitude, longitude, and altitude

    def odomCallBack(self,msg):
        #h = std_msgs.msg.Header()
        #h.stamp = rospy.Time.now()
        #self.geo_pose.header = h
        #self.odom_msg.child_frame_id = self.id
        self.odom_msg = msg
        msg_dict = message_converter.convert_ros_message_to_dictionary(self.odom_msg)
        json_msg = json.dumps({'op':'publish', 'topic': '/rover/odom','msg':msg_dict})
        self.sock.sendto(json_msg.encode(),(self.udp_ip,self.udp_port))

    def poseCallBack(self,msg):
        #h = std_msgs.msg.Header()
        #h.stamp = rospy.Time.now()
        #self.geo_pose.header = h
        #self.odom_msg.child_frame_id = self.id
        self.pose_msg.pose = msg.pose
        msg_dict = message_converter.convert_ros_message_to_dictionary(self.pose_msg)
        json_msg = json.dumps({'op':'publish', 'topic': '/mocap/pose','msg':msg_dict})
        #self.sock.sendto(json_msg.encode(),(self.udp_ip,self.udp_port))



    def advertiseCallBack(self,msg):

        adv_msg = json.dumps({ "op": "advertise",
                                "id": "Truck1",
                                "topic": "/rover/odom",
                                "type": "nav_msgs/Odometry"
                                })

        self.sock.sendto(adv_msg.encode(),(self.udp_ip,self.udp_port))

        adv_msg2 = json.dumps({ "op": "advertise",
                                "id": "Truck1",
                                "topic": "/mocap/pose",
                                "type": "geometry_msgs/PoseStamped"
                                })


        self.sock.sendto(adv_msg2.encode(),(self.udp_ip,self.udp_port))


    def sendCallBack(self,msg):
        msg_dict = message_converter.convert_ros_message_to_dictionary(self.odom_msg)
        json_msg = json.dumps({'op':'publish', 'topic': '/Truck1/odom','msg':msg_dict})
        self.sock.sendto(json_msg.encode(),(self.udp_ip,self.udp_port))

        msg_dict2 = message_converter.convert_ros_message_to_dictionary(self.pose_msg)
        json_msg2 = json.dumps({'op':'publish', 'topic': '/mocap/pose','msg':msg_dict2})
        self.sock.sendto(json_msg2.encode(),(self.udp_ip,self.udp_port))



if __name__ == '__main__':
    rospy.init_node('mocap_Sender')

    mySender = stateSender()

    rospy.spin()
