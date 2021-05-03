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
        self.send_dt = rospy.get_param('send_period',1)
        self.adverise_dt = rospy.get_param('adverise_period',10)
        self.names = rospy.get_param('rigid_bodies/names')
        self.names_n = len(self.names)
        self.sub_list = []
        self.pose_list = []
        self.adv_indx = 0
        self.send_indx = 0

        for current_name in self.names:
            sub = rospy.Subscriber('/qualisys/'+current_name+'/pose', PoseStamped ,self.poseCallBack, current_name)
            self.sub_list.append(sub)
            tmp_pose = PoseStamped()
            self.pose_list.append(tmp_pose)

        self.sock = socket.socket(socket.AF_INET, # Internet
                                 socket.SOCK_DGRAM) # UDP
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)

        self.advertise_timer = rospy.Timer(rospy.Duration(self.adverise_dt/self.names_n),self.advertiseCallBack)
        self.send_timer = rospy.Timer(rospy.Duration(self.send_dt/(self.names_n+1)), self.sendCallBack)
        self.init_time = rospy.get_time()

        self.odom_msg = Odometry()
        self.pose_msg = PoseStamped()

    def poseCallBack(self,msg,name):
        ind = self.names.index(name)
        self.pose_list[ind] = msg
        #self.pose_msg.pose = msg.pose
        #msg_dict = message_converter.convert_ros_message_to_dictionary(self.pose_msg)
        #json_msg = json.dumps({'op':'publish', 'topic': '/mocap/pose','msg':msg_dict})
        #self.sock.sendto(json_msg.encode(),(self.udp_ip,self.udp_port))

    def advertiseCallBack(self,msg):
        status_msg = "Current List of Objects : "
        current_name = self.names[self.adv_indx]
        adv_msg = json.dumps({ "op": "advertise",
                                "id": current_name,
                                "topic": current_name +"/pose",
                                "type": "geometry_msgs/PoseStamped"
                                })
        try:
            self.sock.sendto(adv_msg.encode(),(self.udp_ip,self.udp_port))
        except socket.error as socketerror:
            print("Error: ", socketerror)

        #for current_name in self.names:
        #    status_msg = status_msg + current_name + " "
        #    adv_msg = json.dumps({ "op": "advertise",
        #                            "id": current_name,
        #                            "topic": current_name +"/odom",
        #                            "type": "nav_msgs/Odometry"
        #                            })

        #    self.sock.sendto(adv_msg.encode(),(self.udp_ip,self.udp_port))
        #    rospy.sleep(0.01)
        if(self.adv_indx == (self.names_n-1)):
            self.adv_indx = 0
        else:
            self.adv_indx = self.adv_indx + 1

        rospy.loginfo(status_msg)

    def sendCallBack(self,msg):
        rospy.loginfo(rospy.get_time()-self.init_time)

        current_name = self.names[self.send_indx]

        msg_dict = message_converter.convert_ros_message_to_dictionary(self.pose_list[self.send_indx])
        json_msg = json.dumps({'op':'publish', 'topic':current_name+"/pose",'msg':msg_dict})
        try:
            self.sock.sendto(json_msg.encode(),(self.udp_ip,self.udp_port))
        except socket.error as socketerror:
            print("Error: ", socketerror)

        if(self.send_indx == (self.names_n-1)):
            self.send_indx = 0
        else:
            self.send_indx = self.send_indx + 1
        #for current_name in self.names:
        #    ind = self.names.index(current_name)
        #    msg_dict = message_converter.convert_ros_message_to_dictionary(self.odom_list[ind])
        #    json_msg = json.dumps({'op':'publish', 'topic':current_name+"/odom",'msg':msg_dict})
        #    self.sock.sendto(json_msg.encode(),(self.udp_ip,self.udp_port))
        #    rospy.sleep(0.005)






if __name__ == '__main__':
    rospy.init_node('mocap_pose_sender')

    mySender = mocapSender()

    rospy.spin()
