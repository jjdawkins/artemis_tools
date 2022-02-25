#! /usr/bin/env python3

import rospy
import math
import numpy as np
import socket
import json
import tf

from std_msgs.msg import Empty, String, Header
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry


class odomSender:
    def __init__(self):

        self.udp_ip = rospy.get_param('udp_ip', '192.168.8.255')
        self.udp_port = rospy.get_param('udp_port',9090)
        self.send_dt = rospy.get_param('send_period',1)
        self.adverise_dt = rospy.get_param('adverise_period',10)
        self.names = rospy.get_param('rigid_bodies/names')
        self.names_n = len(self.names)
        self.sub_list = []
        self.pub_list = []
        self.pose_list = []
        self.pos_list = []
        self.old_pos_list = []
        self.vel_est_list = []
        self.old_pose_list = []
        self.last_time_list = []
        self.adv_indx = 0
        self.send_indx = 0


        for current_name in self.names:
            sub = rospy.Subscriber('/vrpn_client_node/'+current_name+'/pose', PoseStamped ,self.poseCallBack, current_name)
            self.sub_list.append(sub)
            pub = rospy.Publisher('/'+current_name+'/odom',Odometry,queue_size=10)
            self.pub_list.append(pub)
            tmp_pose = PoseStamped()
            self.pose_list.append(tmp_pose)
            self.last_time_list.append(rospy.get_time())
            pos = np.zeros([1,3])
            self.pos_list.append(pos)
            old_pos = np.zeros([1,3])
            self.old_pos_list.append(old_pos)
            vel = np.zeros([1,3])
            self.vel_est_list.append(vel)


        self.advertise_timer = rospy.Timer(rospy.Duration(self.adverise_dt/self.names_n),self.advertiseCallBack)
        self.send_timer = rospy.Timer(rospy.Duration(self.send_dt/(self.names_n+1)), self.sendCallBack)
        self.init_time = rospy.get_time()

        self.odom_msg = Odometry()
        self.pose_msg = PoseStamped()

    def poseCallBack(self,msg,name):
        ind = self.names.index(name)

        dt = rospy.get_time() - self.last_time_list[ind]

        q_vrpn = (msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w)

        q_rot = tf.transformations.quaternion_from_euler(math.pi/2,0,math.pi/2)
        q_arty = tf.transformations.quaternion_multiply(q_rot,q_vrpn)
        q_arty_conj = tf.transformations.quaternion_conjugate(q_arty)

        #self.pos_list[ind,0] = msg.pose.position.z
        #self.pos_list[ind,1] = msg.pose.position.x
        #self.pos_list[ind,2] = msg.pose.position.y

        #pos_arty = np.array([msg.pose.position.z,msg.pose.position.x,msg.pose.position.y])

        #pos_vrpn = (msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,0)
        #pos_arty = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q_arty_conj,pos_vrpn),q_arty)
        #euler = tf.transformations.euler_from_quaternion(q_arty)
        #tf.transfor



        #R = tf.transformations.quaternion_matrix(q_arty)
        #pos = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,0])
        #T = np.array([R[0],R[1],R[2]])
        #npos = R*pos
        a = 1/0.2
        self.pos_list[ind][0,:] = np.array([msg.pose.position.z,msg.pose.position.x,msg.pose.position.y])
        #vel = self.pos_list[ind][0,0] - self.old_pos_list[ind][0,0]



        self.vel_est_list[ind][0,:] = (1-a*dt)*self.vel_est_list[ind][0,:] + a*(self.pos_list[ind][0,:] - self.old_pos_list[ind][0,:])
        vel = (self.vel_est_list[ind][0,0],self.vel_est_list[ind][0,1],self.vel_est_list[ind][0,2],0)
        vel_body = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q_arty_conj,vel),q_arty)


        pose = Pose()
        pose.position.x = msg.pose.position.z
        pose.position.y = msg.pose.position.x
        pose.position.z = msg.pose.position.y
        pose.orientation.x = q_arty[0]
        pose.orientation.y = q_arty[1]
        pose.orientation.z = q_arty[2]
        pose.orientation.w = q_arty[3]

        vel_msg = Twist()
        vel_msg.linear.x = vel_body[0]
        vel_msg.linear.y = vel_body[1]
        vel_msg.linear.z = vel_body[2]

        odom = Odometry()
        odom.header.seq = msg.header.seq
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.pose.pose = pose
        odom.twist.twist = vel_msg

        self.pub_list[ind].publish(odom)

        self.old_pos_list[ind][0,:] = self.pos_list[ind][0,:]
        self.last_time_list[ind] = rospy.get_time()




    def advertiseCallBack(self,msg):
        var = 2

    def sendCallBack(self,msg):
        var = 1






if __name__ == '__main__':
    rospy.init_node('artemis_vrpn_odom')

    mySender = odomSender()

    rospy.spin()
