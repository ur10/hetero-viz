#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
import tf


rospy.init_node('goalTransform')
robotNumber = int(rospy.get_param('roboTFNode/name'))
startX = rospy.get_param("roboTFNode/startX")
startY = rospy.get_param("roboTFNode/startY")
pose = Pose().position

pose.x = startX
pose.y = startY


br = tf.TransformBroadcaster()
rospy.sleep(1)

while not rospy.is_shutdown():

    br.sendTransform(
        (pose.x, pose.y, 0.02),
        (0,0,0,1),
        rospy.Time.now(),  
        "/nexus"+str(robotNumber)+"_goal/pole_footprint",
        "/map"
        )
    rospy.sleep(0.05)