#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from marmot.msg import ROS_Task


MSG_THRESHOLD=0.2
REACH_THRESHOLD=0.025
FAST_VEL = 0.25
SLOW_VEL = 0.15
RESOLUTION = 0.5

globalTaskDict = dict()
globalTaskQueue = list()

OFFSET = [4.5,0]

globalCurrentTask = -1

def resetCallback(data):
    global globalTaskQueue, globalTaskDict, globalCurrentTask, goal, startX, startY

    globalTaskDict = dict()
    globalTaskQueue = list()

    globalCurrentTask = -1

    goal.x = startX
    goal.y = startY

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z

def odomCallback(data):
    global pose

    pose.x=-data.pose.position.y+OFFSET[0]
    pose.y=data.pose.position.x+OFFSET[1]

    # pose.x +=startX
    # pose.y +=startY

    orientation_q = data.pose.orientation
    roll, pitch, pose.z = euler_from_quaternion (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    # pose.z+= startYaw
    
    # print(pose)

def commsCallback(data):
    global globalTaskQueue, globalTaskDict, robotNumber, globalCurrentTask
    # print(robotName, data.taskID, data.status, globalTaskQueue)

    # print(data)

    if(data.status==-3 or globalCurrentTask==-3):
        globalCurrentTask = -3
        return

    if(data.status==201 and data.taskID not in globalTaskQueue and data.taskID>globalCurrentTask):
        # print(robotName, "Adding to queue", data.taskID)
        globalTaskDict[data.taskID] = data
        globalTaskQueue.append(data.taskID)
        if(globalCurrentTask==-1):
            nextTask()
    # print('done')

def l2(pose, goal):
        pose = np.array([pose.x, pose.y, pose.z])
        goal = np.array([goal.x, goal.y, goal.z])
        ans = np.linalg.norm(pose-goal, 2)  # L2 norm
        # print(pose, goal, ans)
        return ans

def nextTask():
    global globalCurrentTask, commsPub, robotNumber, goal
    if len(globalTaskQueue)!=0:
        globalCurrentTask = globalTaskQueue.pop(0)
        action = globalTaskDict[globalCurrentTask].action
        if action ==2:
        # rospy.loginfo("up")
            goal.x = goal.x+RESOLUTION
        
        elif action == 3:
            # rospy.loginfo("right")
            goal.y = goal.y-RESOLUTION


        elif action ==4:
            # rospy.loginfo("down")
            goal.x = goal.x-RESOLUTION

        elif action==1:
            # rospy.loginfo("left")
            goal.y = goal.y+RESOLUTION
    # else:
    #     globalCurrentTask = -1

def doneTask():
    global globalCurrentTask, robotNumber, commsPub, pose, goal
    if(globalCurrentTask==-3):
        return
    if(l2(pose, goal)<=MSG_THRESHOLD):
        if(globalCurrentTask!=-1):
            message = ROS_Task()
            message.taskID = globalCurrentTask
            message.action = globalTaskDict[globalCurrentTask].action
            message.robotID = robotNumber
            message.status = 202

            commsPub.publish(message)

            nextTask()
        else:
            message = ROS_Task()
            message.taskID = globalCurrentTask
            message.robotID = robotNumber
            commsPub.publish(message)
        


if __name__ == '__main__':

    rospy.init_node('roboPID')

    robotNumber = int(rospy.get_param('roboNode/name'))
    realRobotNumber = int(rospy.get_param('roboNode/RealID'))
    startX = rospy.get_param('roboNode/startX')
    startY = rospy.get_param('roboNode/startY')
    
    robotName = "Robo"+str(realRobotNumber)

    pose = Odometry().pose.pose.position
    goal = Odometry().pose.pose.position

    goal.x = startX
    goal.y = startY

    velPub = rospy.Publisher("/"+robotName+"/cmd_vel", Twist, queue_size=1)
    commsPub = rospy.Publisher("/comms_nexus"+str(robotNumber),ROS_Task, queue_size=10000)
    goalSub = rospy.Subscriber("/comms_nexus"+str(robotNumber),ROS_Task, commsCallback, queue_size=10000)

    odomSub = rospy.Subscriber("/vrpn_client_node/RigidBody"+str(realRobotNumber)+"/pose", PoseStamped, odomCallback)
    resetSub = rospy.Subscriber("/reset",Twist, resetCallback, queue_size=10)

    diff_x = 0
    diff_y = 0

    rospy.loginfo(robotName+" Ready")

    time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        # print(globalCurrentTask)
        vel = Twist()
        diff_x = goal.x-pose.x
        diff_y = goal.y-pose.y
        # print(diff_x, diff_y)

        if abs(diff_x)>MSG_THRESHOLD:
            vel.linear.x = FAST_VEL*(diff_x/abs(diff_x))
        elif abs(diff_x)>REACH_THRESHOLD:
            vel.linear.x = (((FAST_VEL-SLOW_VEL)*(diff_x-REACH_THRESHOLD))/(MSG_THRESHOLD-REACH_THRESHOLD))+SLOW_VEL*(diff_x/abs(diff_x))
        else:
            pass
    
        if abs(diff_y)>MSG_THRESHOLD:
            vel.linear.y = FAST_VEL*(diff_y/abs(diff_y))
        elif abs(diff_y)>REACH_THRESHOLD:
            vel.linear.y = (((FAST_VEL-SLOW_VEL)*(diff_y-REACH_THRESHOLD))/(MSG_THRESHOLD-REACH_THRESHOLD))+SLOW_VEL*(diff_y/abs(diff_y))
        else:
            pass

        if abs(pose.z)>=np.pi/36:
            vel.angular.z = -FAST_VEL*(pose.z/abs(pose.z))


        doneTask()
    
        # introduce Error        
        # vel.linear.x =  vel.linear.x + VEL_ERROR_ABSOLUTE*np.random.uniform(-1,1)
        # vel.linear.y =  vel.linear.y + VEL_ERROR_ABSOLUTE*np.random.uniform(-1,1)
        # print(vel)
        # print(pose)
        # print(goal)
        velPub.publish(vel)

        rospy.sleep(0.01)
    