#! /usr/bin/env python3

import rospy
import math
import numpy as np

from ackermann_msgs.msg import *
from std_msgs.msg import Empty, String, Header, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
import tf
from tf_conversions.transformations import quaternion_to_euler


class BebopControl:
    def __init__(self):

        self.max_str_angle = rospy.get_param('max_str_angle',0.6)
        self.max_speed = rospy.get_param('max_speed',2)
        self.L = rospy.get_param('wheel_base',0.2) # 0.2 meters
        self.KpSpd = rospy.get_param('Kp_spd',20)
        self.KiSpd = rospy.get_param('Ki_spd',1)

        self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=5)

        self.pose_sub = rospy.Subscriber('odom',Odometry,self.odomCallBack)
        self.joy_sub = rospy.Subscriber('joy',Joy,self.joyCallBack)
        self.send_timer = rospy.Timer(rospy.Duration(0.1), self.sendCallBack)

        self.yawRate = 0.0
        self.desSpeed = 0.0
        self.spdErrInt = 0
        self.pos = [0,0,0]


    def odomCallBack(self,msg):
        q = Quaternion()
        q.x = msg.pose.pose.orientiation.x
        q.y = msg.pose.pose.orientiation.y
        q.z = msg.pose.pose.orientiation.z
        q.w = msg.pose.pose.orientiation.w

        euler = tf.transformations.euler_from_quaternion(q)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        speedx = msg.twist.twist.linear.x
        speedy = msg.twist.twist.linear.y
        self.speed = math.sqrt(speedx*speedx + speedy*speedy)

    def joyCallBack(self,msg):
        msg.axes[0]


    def sendCallBack(self,msg):


        cmd_msg = Twist()

        cmd_g[0] = Kp_xy*(self.des_pos[0] - self.pos[0])
        cmd_g[0] = Kp_xy*(self.des_pos[0] - self.pos[0])
        cmd_g = Kp_z*(self.des_pos[2] - self.pos[2])

        cmd.linear.y
        cmd.linear.z
        cmd.angular.z = K_yaw*(self.des_yaw - self.yaw)
        # speed controller (Using PI controller on acceleration)
        self.spdErr = self.desSpeed - self.speed

        # PWM duty cycle for throttle control
        self.thr_cmd = self.KpSpd*self.spdErr + self.KiSpd*self.spdErrInt # PI controller

        # Saturate speed command at 20% duty cycle
        #if self.thr_cmd > 0.2:
        #    self.thr_cmd = 0.2
        #elif self.thr_cmd < -0.2:
        #    self.thr_cmd = -0.2

        # update speed error integral term
        self.spdErrInt = self.spdErrInt + self.spdErr*0.1 # 10 Hz sampling rate
        # saturate speed error integral term at 2
        #if self.spdErrInt > 2:
        #    self.spdErrInt = 2.0

        # package ackermann message
        self.ackMsg.drive.steering_angle = steerAng
        self.ackMsg.drive.acceleration = self.thr_cmd

        rospy.loginfo("%f,%f",self.speed,self.thr_cmd)
        # publish ackermann message
        self.cmd_pub.publish(self.ackMsg)



if __name__ == '__main__':
    rospy.init_node('tracking_controller')

    myRover = roverControl()

    rospy.spin()
