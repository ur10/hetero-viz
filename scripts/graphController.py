#!/usr/bin/env python3

#
# import rospy
# import numpy as np
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# # from marmot.msg import ROS_Task
# import tf
# import pickle
# from task_env import TaskEnv
#
# START=[(2, 7),(20, 1),(17, 16),(2, 0),(10, 1),(16, 22),(17, 7)]
# VEL_SCALE = 0.1
# AGENT_NUMS = 1
# testSet = "/home/ur10/mapf_ws/testSet_simulation"
#
# def velCallback(data):
#     global vel
#     vel = data
#     print('HERE')
#
# def get_goal_velocity(goal, current_pose):
#      a = 1
#      angle = np.arctan2(goal.y - current_pose.y, goal.x - current_pose.x)
#      vel = Twist()
#      vel.linear.x = 0.2 * np.cos(angle)
#      vel.linear.y = 0.2 * np.sin(angle)
#      return  vel
#
# if __name__ == '__main__':
#
#     rospy.init_node('roboPID')
#     rospy.sleep(1)
#     # rospy.spin()
#     # try:
#     # robotNumber = int(rospy.get_param('roboNode/name'))
#     # startX = rospy.get_param('roboNode/startX')
#     # startY = rospy.get_param('roboNode/startY')
#     #
#     # robotName = "nexus" + str(robotNumber)
#
#     # try:
#     robotNumber = 1
#     # robotNumber = int(rospy.get_param('roboNode/name'))
#
#     # startX = rospy.get_param('roboNode/startX')
#     # startY = rospy.get_param('roboNode/startY')
#
#     robotName = "nexus"+str(robotNumber)
#     #
#     # rospy.loginfo(" Ready")
#     print("READY")
#     #
#     velSubs = []
#     velPubs = []
#     velocities = []
#
#     for i in range(AGENT_NUMS):
#         vel = Twist()
#         velocities.append(vel)
#
#     velSub = rospy.Subscriber("/" + robotName + "/cmd_vel", Twist, velCallback)
#
#     for i in range(AGENT_NUMS):
#         Sub = rospy.Subscriber("/"+f"nexus{i}"+"/cmd_vel", Twist, velCallback)
#         Pub = rospy.Publisher("/"+f"nexus{i}"+"/cmd_vel", Twist, queue_size=1)
#         velPubs.append(Pub)
#         velSubs.append(Sub)
#
#
#     poses = []
#
#     br = tf.TransformBroadcaster()
#
#     for i in range(AGENT_NUMS):
#         pose = Odometry().pose.pose.position
#         pose.x = START[i][0]
#         pose.y = START[i][0]
#         poses.append(pose)
#
#     diff_x = 0
#     diff_y = 0
#
#     time = rospy.Time.now().to_sec()
#
#     env = pickle.load(open(f'{testSet}/env_0/RL.pkl', 'rb'))
#
#     task_locations = [env['tasks'][i]['location'] for i in range(len(env['tasks']))]
#     agent_locations = []
#
#     for i in range(AGENT_NUMS):
#
#         # location  = [route for j in range(len(env['agent'][i]['route'])) for route in task_locations[env['agent'][j]['route']] ]
#         location = []
#         for route in env['agent'][i]['route'][1:-1]:
#             location.append(task_locations[route])
#         agent_locations.append(location)
#
#
#     while not rospy.is_shutdown():
#
#         for i in range(AGENT_NUMS):
#             poses[i].x += velocities[i].linear.x * VEL_SCALE
#             poses[i].y += velocities[i].linear.y * VEL_SCALE
#
#             if poses[i].x - agent_locations[i][0][0] < 0.1 and poses[i].y -agent_locations[i][0][1] < 0.1:
#                 vel_pub = Twist()
#                 vel_pub.linear.x= 0
#                 vel_pub.linear.y = 0
#                 # velPubs[i].publish(vel_pub)
#                 agent_locations[i].pop(0)
#             else:
#                 goal_pose = Odometry().pose.pose.position
#                 goal_pose.x = agent_locations[i][0][0]
#                 goal_pose.y = agent_locations[i][0][1]
#                 vel_pub = get_goal_velocity(goal_pose, poses[i])
#
#         for i in range(AGENT_NUMS):
#             br.sendTransform(
#                 (poses[i].x, poses[i].y, 0.07),
#                 (0,0,0,1),
#                 rospy.Time.now(),
#                 "/nexus"+str(i)+"/base_footprint",
#                 "/map"
#                 )
#
#         rospy.sleep(0.05)


import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from marmot.msg import ROS_Task
import tf
from task_env import TaskEnv
import pickle
import os
VEL_SCALE = 1


def velCallback(data):
    global vel
    vel = data
    # print("HERE")



testSet = f"{os.path.expanduser('~')}/mapf_ws/testSet_simulation"
# START=[(1, 1),(2, 2),(3, 3),(4, 4),(5, 5),(6, 6),(7, 7),(1, 4),(5, 5),(17, 18),(7, 22),(21, 20),(7, 1),(1, 2),(15, 21),(2, 17),(5, 10),(17, 13),(10, 22),(2, 21),(8, 4),(8, 19),(5, 3),(20, 3),(14, 12),(19, 20),(1, 7),(2, 14),(14, 21),(15, 2),(11, 21),(17, 1),(2, 10),(5, 9),(1, 5),(13, 1),(22, 2),(15, 14),(3, 21),(2, 8),(1, 0),(22, 16),(0, 14),(21, 2),(8, 22),(18, 8),(18, 1),(0, 20),(1, 22),(17, 5),(11, 8),(17, 11),(8, 17),(0, 9),(16, 0),(17, 2),(4, 8),(14, 19),(2, 11),(12, 14),(8, 6),(21, 1),(2, 16),(21, 17),(0, 18),(19, 22),(20, 16),(9, 2),(0, 17),(7, 8),(1, 6),(11, 0),(17, 3),(0, 0),(9, 14),(22, 10),(8, 9),(21, 16),(21, 15),(1, 1),(20, 15),(19, 1),(9, 22),(22, 3),(2, 9),(7, 21),(14, 13),(0, 21),(12, 8),(0, 2),(18, 20),(17, 22),(1, 10),(0, 10),(5, 4),(22, 5),(13, 8),(2, 22),(5, 15),(6, 14),(1, 13),(5, 19),(22, 7),(0, 12),(15, 22),(21, 19),(5, 20),(10, 8),(21, 7),(22, 0),(7, 2),(1, 16),(11, 19),(14, 8),(8, 5),(1, 19),(1, 11),(0, 5),(1, 20),(8, 7),(5, 18),(14, 6),(22, 21),(14, 17),(6, 2),(18, 14),(15, 20),(12, 22),(21, 12),(6, 8),(0, 7),(11, 17),(22, 20),(0, 11),(11, 6),(14, 2),(2, 3),(16, 2),(14, 10),(0, 13),(16, 1),(15, 1),(21, 21),(18, 21),(0, 4),(5, 22),(20, 6),(1, 9),(20, 14),(12, 0),(0, 8),(20, 5),(20, 13),(12, 2),(13, 22),(1, 3),(21, 10),(11, 3),(4, 1),(17, 21),(20, 18),(14, 3),(14, 14),(0, 6),(7, 14),(22, 8),(8, 1),(2, 19),(12, 1),(20, 4),(18, 0),(1, 17),(5, 1),(22, 18),(19, 0),(22, 9),(9, 8),(17, 17),(2, 12),(21, 8),(8, 21),(20, 19),(5, 11),(17, 10),(17, 6),(22, 14),(3, 20),(11, 15),(3, 2),(8, 3),(17, 15),(13, 21),(18, 22),(14, 1),(21, 3),(6, 0),(8, 8),(14, 11),(21, 13),(11, 1),(10, 20),(22, 13),(5, 13),(6, 21),(0, 22),(3, 22),(0, 16),(3, 14),(4, 14),(1, 15),(5, 14),(8, 15)]
env = pickle.load(open(f'{testSet}/env_0/baseline.pkl', 'rb'))
START = []

for i in range(0,3):
    env['agent'][i] = env['agent'][i+3]
    del env['agent'][i+3]
# self.env = env

for i in range(len(env['agent'])):
    task_dict = {'current_agent_num': 0, 'required_agent_num': max(env['tasks'][i]['requirements']),
                 'task_time': env['tasks'][i]['time']}
    # self.task_track.append(task_dict)
    START.append(env['agent'][i]['depot'] *10 + 0.1*i )

if __name__ == '__main__':

    rospy.init_node('roboPID')
    rospy.sleep(1)
    # rospy.spin()
    # try:
    robotNumber = int(rospy.get_param('roboNode/name'))
    startX = rospy.get_param('roboNode/startX')
    startY = rospy.get_param('roboNode/startY')

    robotName = "nexus" + str(robotNumber)

    rospy.loginfo(robotName + " Ready")
    Pub = rospy.Publisher("/" + f"nexus{robotNumber}" + "/cmd_vel", Twist, queue_size=1)

    stat_pub = rospy.Publisher("/" + f"nexus{robotNumber}" + "/status_check", Bool, queue_size=1)
    velSub = rospy.Subscriber("/" + robotName + "/cmd_vel", Twist, velCallback)

    pose = Odometry().pose.pose.position
    vel = Twist()
    br = tf.TransformBroadcaster()

    # pose.x = startX
    # pose.y = startY
    print(START)
    pose.x = START[robotNumber][0]
    pose.y = START[robotNumber][1]
    diff_x = 0
    diff_y = 0

    time = rospy.Time.now().to_sec()
    current_time = time

    while not rospy.is_shutdown():
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2
        cmd_vel.linear.y = 0.01
        time = rospy.Time.now().to_sec()
        time_diff = abs(current_time-time)
        current_time = time
        # Pub.publish(cmd_vel)
        pose.x += vel.linear.x * time_diff
        pose.y += vel.linear.y * time_diff


        stat_msg= Bool()
        stat_msg.data = False
        stat_pub.publish(stat_msg)
        # if robotNumber == 2:
            # print(f'current pose is {pose}')
        br.sendTransform(
            (pose.x, pose.y, 0.07),
            (0, 0, 0, 1),
            rospy.Time.now(),
            "/nexus" + str(robotNumber) + "/base_footprint",
            "/map"
        )
        # rospy.spin_once()
        rospy.sleep(0.09)
