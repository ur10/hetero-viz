#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import tf

pose = None

OFFSET = [0,0]

def cb(data):
    global pose
    pose = data.pose

rospy.init_node('roboPID')
robotNumber = int(rospy.get_param('roboTFNode/name'))
robotRealNumber = int(rospy.get_param('roboTFNode/RealID'))
robotName = "vrpn_client_node/RigidBody"
odomSub = rospy.Subscriber("/"+robotName+str(robotRealNumber)+"/pose", PoseStamped, cb)
br = tf.TransformBroadcaster()
rospy.sleep(1)

while not rospy.is_shutdown():
    if pose is not None:

        br.sendTransform(
            (-pose.position.y, pose.position.x, -0.07),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            rospy.Time.now(),  
            "/nexus"+str(robotNumber)+"/base_footprint",
            "/optitrack"
            )
    rospy.sleep(0.05)