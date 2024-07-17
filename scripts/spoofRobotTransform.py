#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Twist


SCALE = 0.01

def cb(data):
    global pose
    pose.x+=SCALE*data.linear.x
    pose.y+=SCALE*data.linear.y

rospy.init_node('roboTransform')
robotNumber = int(rospy.get_param('roboTFNode/name'))
startX = rospy.get_param("roboTFNode/startX")
startY = rospy.get_param("roboTFNode/startY")
pose = Pose().position

pose.x = startX
pose.y = startY

velSub = rospy.Subscriber("/nexus"+str(robotNumber)+"/cmd_vel", Twist, cb)

br = tf.TransformBroadcaster()
ps = rospy.Publisher("/nexus"+robotNumber+"/odom",Pose, queue_size=10000)

rospy.sleep(1)

while not rospy.is_shutdown():

    br.sendTransform(
        (pose.x, pose.y, 0.07),
        (0,0,0,1),
        rospy.Time.now(),  
        "/nexus"+str(robotNumber)+"/base_footprint",
        "/map"
        )
    ps.publish(pose)
    rospy.sleep(0.05)