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


class mocapSender:
    def __init__(self):

        self.udp_ip = rospy.get_param('udp_ip', '192.168.8.255')
        self.udp_port = rospy.get_param('udp_port',9090)
        self.send_dt = rospy.get_param('send_period',0.04)
        self.adverise_dt = rospy.get_param('adverise_period',10)
        self.names = rospy.get_param('rigid_bodies/names')
        self.names_n = len(self.names)
        self.sub_list = []
        self.odom_list = []

        for current_name in self.names:
            sub = rospy.Subscriber('/qualisys/'+current_name+'/odom', Odometry,self.odomCallBack, current_name)
            self.sub_list.append(sub)
            tmp_odom = Odometry()
            self.odom_list.append(tmp_odom)

        self.sock = socket.socket(socket.AF_INET, # Internet
                                 socket.SOCK_DGRAM) # UDP
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)

        self.advertise_timer = rospy.Timer(rospy.Duration(self.adverise_dt),self.advertiseCallBack)
        self.send_timer = rospy.Timer(rospy.Duration(self.send_dt), self.sendCallBack)

        self.odom_msg = Odometry()
        self.pose_msg = PoseStamped()

    def odomCallBack(self,msg,name):
        ind = self.names.index(name)
        self.odom_list[ind] = msg

    def poseCallBack(self,msg):
        self.pose_msg.pose = msg.pose
        msg_dict = message_converter.convert_ros_message_to_dictionary(self.pose_msg)
        json_msg = json.dumps({'op':'publish', 'topic': '/mocap/pose','msg':msg_dict})
        #self.sock.sendto(json_msg.encode(),(self.udp_ip,self.udp_port))

    def advertiseCallBack(self,msg):
        status_msg = "Current List of Objects : "
        for current_name in self.names:
            status_msg = status_msg + current_name + " "
            adv_msg = json.dumps({ "op": "advertise",
                                    "id": current_name,
                                    "topic": current_name +"/odom",
                                    "type": "nav_msgs/Odometry"
                                    })

            self.sock.sendto(adv_msg.encode(),(self.udp_ip,self.udp_port))
            rospy.sleep(0.01)

        rospy.loginfo(status_msg)

    def sendCallBack(self,msg):
        for current_name in self.names:
            ind = self.names.index(current_name)
            msg_dict = message_converter.convert_ros_message_to_dictionary(self.odom_list[ind])
            json_msg = json.dumps({'op':'publish', 'topic':current_name+"/odom",'msg':msg_dict})
            self.sock.sendto(json_msg.encode(),(self.udp_ip,self.udp_port))
            rospy.sleep(0.01)



if __name__ == '__main__':
    rospy.init_node('mocap_Sender')

    mySender = mocapSender()

    rospy.spin()
