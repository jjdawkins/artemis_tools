#! /usr/bin/env python3

import rospy
import math
import numpy as np

from std_msgs.msg import Empty, String, Header, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseArray


def wyptPublisher():
    pub = rospy.Publisher('waypoint',PoseArray,queue_size=10)
    rospy.init_node('waypointPublisher', anonymous=True)
    rate = rospy.Rate(5) # Hz



    while not rospy.is_shutdown():
        wypts = PoseArray()
        wypts.header.stamp = rospy.Time.now()
        pts = np.zeros((2,4))
        pts[0,:] = [2, 3, 3, -1]
        pts[1,:] = [2, 2, 3, 3]
        for ii in range(0,np.size(pts,1),1):
            pt = Pose()
            pt.position.x = pts[0,ii]
            pt.position.y = pts[1,ii]
            #rospy.loginfo(ii)
            #wypts.poses[ii] = pt
            wypts.poses.append(pt)
            #rospy.loginfo(pt)

        #rospy.loginfo(wypts)
        pub.publish(wypts)
        rate.sleep()



if __name__ == '__main__':
    try:
        wyptPublisher()
    except rospy.ROSInterruptException:
        pass
