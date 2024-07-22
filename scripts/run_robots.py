#!/usr/bin/env python
import rosnode
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pickle
from task_env import TaskEnv
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import os
from std_msgs.msg import  Bool
#TODO - 1. CHECK THE WAITING TIME 2. CHECK THE TASK REQUIREMENT NUMBER 3. PUBLISH MARKERS AT THE CORRECT PLACE 4. CORRECT GOAL POSE
VEL_SCALE  = 1
testSet = f"{os.path.expanduser('~')}/mapf_ws/testSet_simulation"
AGENT_NUMS = 6
LEFT = [0, 0]
RESOLUTION = 0.5
class MultiRobotController:
    def __init__(self):
        rospy.init_node('multi_robot_controller', anonymous=True)

        # Publisher for each robot's cmd_vel
        self.vel_pubs = []
        self.positions = []
        self.agent_locations = []
        self.start_positions = []
        self.agents_track = []
        self.task_track = []
        self.status_check = [False]*AGENT_NUMS
        self.node_waiting = True

        env = pickle.load(open(f'{testSet}/env_0/baseline.pkl', 'rb'))
        self.env = env
        task_locations = [env['tasks'][i]['location']*10 + 0.1*i for i in range(len(env['tasks']))]
        print(f'the task locations are {task_locations}')
        self.task_locations = task_locations
        for i in range(len(env['tasks'])):
            task_dict = {'current_agent_num': 0, 'required_agent_num': (len(env['tasks'][i]['members'])), 'task_time': env['tasks'][i]['time'], 'members': env['tasks'][i]['members']}
            self.task_track.append(task_dict)

        for i in range(AGENT_NUMS):
            agent_dict = {'task_idx': 0, 'task_num': 0, 'route': [],'waiting': False, 'task_start_time': rospy.Time.now().to_sec(), 'current_position': [0, 0], 'first_arrival': True, 'travelling': False, 'odom_time': rospy.Time.now().to_sec()}
            agent_dict['route'] = env['agent'][i]['route'][1:-1]
            agent_dict['task_num'] = agent_dict['route'][0]
            self.agents_track.append(agent_dict)
            self.start_positions.append(env['agent'][i]['depot'] *10 +0.1*i)


        for i in range(AGENT_NUMS):

            # location  = [route for j in range(len(env['agent'][i]['route'])) for route in task_locations[env['agent'][j]['route']] ]
            location = []
            for route in env['agent'][i]['route'][1:-1]:
                location.append(task_locations[route])
            location.append([self.start_positions[i][0],self.start_positions[i][1]])
            self.agent_locations.append(location)


        for i in range(0, AGENT_NUMS):
            pub = rospy.Publisher(f'/nexus{i}/cmd_vel', Twist, queue_size=10)
            self.vel_pubs.append(pub)

        # Subscriber for each robot's odometry
        self.odom_subs = []
        self.status_subs = []


        for i in range(0,AGENT_NUMS):
            pose = Odometry().pose.pose.position
            pose.x = self.start_positions[i][0]
            pose.y = self.start_positions[i][1]

            self.positions.append(pose)
        for i in range(0, AGENT_NUMS):
            sub = rospy.Subscriber(f'/nexus{i}/cmd_vel', Twist, self.odom_callback, callback_args=i)
            # stat_sub = rospy.Subscriber( f"/nexus{i}/status_check", Bool,self.status_callback,  callback_args=i)
            # self.status_subs.append(stat_sub)
            self.odom_subs.append(sub)
        self.node_check_sub = rospy.Subscriber( f"/nexus{AGENT_NUMS-1}/status_check", Bool,self.status_callback)
        self.marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        # To store the position of each robot
        # self.positions = {}

    def status_callback(self, msg):
        self.node_waiting = msg.data
        # print('in the status callback')
    def get_goal_velocity(self, goal, current_pose):
         a = 1
         angle = np.arctan2(goal.y - current_pose.y, goal.x - current_pose.x)
         vel = Twist()
         vel.linear.x = 0.6 * np.cos(angle)
         vel.linear.y = 0.6 * np.sin(angle)
         return  vel

    def odom_callback(self, vel, robot_id):
        # Update the position of the corresponding robot
        # self.positions[robot_id] = msg.pose.pose.position
        current_time = rospy.Time.now().to_sec()
        time_diff = abs(self.agents_track[robot_id]['odom_time'] - current_time)
        self.agents_track[robot_id]['odom_time'] = current_time
        self.positions[robot_id].x += vel.linear.x * time_diff
        self.positions[robot_id].y += vel.linear.y * time_diff
        # print(f"robot {robot_id}")
        # rospy.loginfo(f"Robot {robot_id} position: {self.positions[robot_id].x, self.positions[robot_id].y}")

    def publish_velocity_commands(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            twist = Twist()
            twist.linear.x = 0.5  # Example linear velocity
            twist.angular.z = 0.1  # Example angular velocity
            print(self.status_check)
            for i in range(AGENT_NUMS):
                # if all(agent == True for agent in self.status_check):
                #     print('Inside the loop')
                if True:
                    # poses[i].x += velocities[i].linear.x * VEL_SCALE
                    # poses[i].y += velocities[i].linear.y * VEL_SCALE
                    curr_idx = self.agents_track[i]['task_idx']

                       # if self.agents_track[i]['waiting'] == True:
                    #     vel_pub = Twist()
                    #     vel_pub.linear.x = 0
                    #     vel_pub.linear.y = 0
                    #     print('waiting top')
                    #     self.vel_pubs[i].publish(vel_pub)


                    if curr_idx == len(self.agent_locations[i]): # goto task is the depot
                        goal_pose = Odometry().pose.pose.position
                        goal_pose.x = self.start_positions[i][0]
                        goal_pose.y = self.start_positions[i][1]
                        # print(f" ROBOT {i} - !!! GOING TO THE Position - {goal_pose}")
                        vel_pub = self.get_goal_velocity(goal_pose, self.positions[i])
                        # print(f"published velocit {vel_pub}")
                        self.vel_pubs[i].publish(vel_pub)

                    elif abs(self.positions[i].x - self.agent_locations[i][curr_idx][0]) < 0.08 and abs(self.positions[i].y - self.agent_locations[i][curr_idx][1]) < 0.08:
                        vel_pub = Twist()
                        vel_pub.linear.x = 0
                        vel_pub.linear.y = 0
                        self.vel_pubs[i].publish(vel_pub)

                        if self.agents_track[i]['waiting'] == False:
                            task_idx = self.agents_track[i]['task_num']
                            # self.agents_track[i]['task_start_time'] = rospy.Time.now().to_sec()
                            self.agents_track[i]['waiting'] = True
                            self.task_track[task_idx]['current_agent_num'] +=1
                            print(f" ROBOT {i} - !!! Arrived at the task - {self.agents_track[i]['task_num']} for the first time")
                        else:

                            # print(f'agent {i} waiting time {work_time}')

                            task_idx = self.agents_track[i]['task_num']
                            current_agent_count = self.task_track[task_idx]['current_agent_num']
                            required_agent_count = self.task_track[task_idx]['required_agent_num']
                            print(
                                f" ROBOT {i} - !!! WAITING ON THE task - {self.agents_track[i]['task_num']}, remaining agents to arrive - {required_agent_count - current_agent_count}")

                            if current_agent_count >= required_agent_count and self.agents_track[i]['first_arrival'] == True:
                                self.agents_track[i]['first_arrival'] = False
                                self.agents_track[i]['task_start_time'] = rospy.Time.now().to_sec()

                            elif current_agent_count >= required_agent_count and self.agents_track[i]['first_arrival'] == False:
                                work_time = abs(self.agents_track[i]['task_start_time'] - rospy.Time.now().to_sec())
                                print(f'Agent{i} The time for working is {work_time}')
                                if work_time > 3.5:

                                    self.agents_track[i]['task_idx'] += 1 # (2.7 , 11.4), (-3.2, 7.5),(4.95,6.6),
                                    if self.agents_track[i]['task_idx'] < len(self.agents_track[i]['route']):
                                        self.agents_track[i]['task_num'] = self.agents_track[i]['route'][self.agents_track[i]['task_idx']]
                                    self.agents_track[i]['waiting'] = False


                                    print('waitinng bottom')
                                vel_pub = Twist()
                                vel_pub.linear.x = 0
                                vel_pub.linear.y = 0
                                # self.vel_pubs[i].publish(vel_pub)
                            # elif work_time < 2.5 and current_agent_count < required_agent_count:

                        #
                        # self.task_track[self.agents_track[i]['task_num']]['current_agent_num'] += 1
                        # self.agents_track[i]['current_waiting_time'] = rospy.Time.now().to_sec()
                        #
                        # current_agent_num = self.task_track[self.agents_track[i]['task_num']]['current_agent_num'] + 1
                        # self.task_track[self.agents_track[i]['task_num']]['current_agent_num'] = current_agent_num
                        # require_agent_num = self.task_track[self.agents_track[i]['task_num']]['required_agent_num']
                        # print(f"Required agent num {require_agent_num}, current agent num {current_agent_num}")
                        #
                        # if current_agent_num < self.task_track[self.agents_track[i]['task_num']]['required_agent_num']:
                        #     vel_pub = Twist()
                        #     vel_pub.linear.x = 0
                        #     vel_pub.linear.y = 0
                        #     self.vel_pubs[i].publish(vel_pub)
                        #     print(f"Reached the task number {self.agents_track[i]['task_num']} but not enough agents")
                        #     self.agents_track[i]['waiting'] = True
                        # elif (current_agent_num == self.task_track[self.agents_track[i]['task_num']]['required_agent_num']) and self.agents_track[i]['waiting'] == True:
                        #     # All the agents have arrived and we should start the work on the task
                        #     self.agents_track[i]['task_start_time'] = rospy.Time.now().to_sec()
                        #     self.agents_track[i]['waiting'] = False
                        #
                        #     vel_pub = Twist()
                        #     vel_pub.linear.x = 0
                        #     vel_pub.linear.y = 0
                        #     self.vel_pubs[i].publish(vel_pub)
                        #     print(f"Reached the task number {self.agents_track[i]['task_num']} and enough agents, beginning the task")
                        # else:
                        #     work_time = abs(self.agents_track[i]['task_start_time'] - rospy.Time.now().to_sec())
                        #     print(f'agent work time {work_time}')
                        #     if work_time > self.task_track[self.agents_track[i]['task_num']]['task_time']:
                        #         self.agents_track[i]['task_idx'] += 1
                        #         self.agents_track[i]['task_num'] = self.agents_track[i]['route'][self.agents_track[i]['task_idx']]
                        #         required_working_time = {self.task_track[self.agents_track[i]['task_num']]['task_time']}
                        #         print(f'agent work time is {work_time} and task requirement is {required_working_time}')
                        #     else:
                        #         vel_pub = Twist()
                        #         vel_pub.linear.x = 0
                        #         vel_pub.linear.y = 0
                        #         self.vel_pubs[i].publish(vel_pub)
                        # if len(self.agent_locations[i]) > 1:
                        #     self.agent_locations[i].pop(0)
                        # else:
                        #     print(f" ROBOT {i} - !!! GOING TO THE ORIGINAL DEPOT")
                        #     self.agent_locations[i][0][0] = self.start_positions[i][0]
                        #     self.agent_locations[i][0][1] = self.start_positions[i][1]
                        #     print(f"The current location is {self.positions[i]} and the goal position is {self.agent_locations[i][0]}")
                        # print(f"published velocit {vel_pub}")
                    else:
                        self.agents_track[i]['travelling'] = True
                        self.agents_track[i]['first_arrival'] = True
                        goal_pose = Odometry().pose.pose.position
                        goal_pose.x = self.agent_locations[i][curr_idx][0]
                        goal_pose.y = self.agent_locations[i][curr_idx][1]
                        # print(f" ROBOT {i} - !!! GOING TO THE task - {self.agents_track[i]['task_num']}")
                        # print(f" ROBOT {i} - !!! goal pose - {goal_pose} \n current pose - {self.positions[i]}")
                        vel_pub = self.get_goal_velocity(goal_pose, self.positions[i])
                        # print(f"published velocit {vel_pub}")
                        self.vel_pubs[i].publish(vel_pub)

            # Publish the same velocity command to each robot
            # for pub in self.vel_pubs:
            #     pub.publish(twist)
            self.publish_markers()
            rate.sleep()

    def publish_markers(self):
        marker_array = MarkerArray()
        markerpose = [(5.02, 6.73), (1.80, 11.98), (1.87, 11.35), (0.59, 6.39), (-1.36, 8.76), (-3.45, 7.89),
                      (2.19, 9.99), (1.40, 10.13), (-1.06, 11.57), (-0.11, 8.93),
                      (-2.12, 8.03), (4.44, 7.18), (0.98, 4.27), (0.55, 3.32), (0.27, 3.74)  ]
        for i in range(len(self.task_locations)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "target_positions"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            coords = self.getCoord(self.task_locations[i])
            marker.pose.position.x = self.task_locations[i][0]
            marker.pose.position.y = self.task_locations[i][1]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
            # print(f"marker position ={marker.pose.position} ")
    def getCoord(self, i):
      return [LEFT[0]+i[1]*RESOLUTION+RESOLUTION/2, LEFT[1]+i[0]*RESOLUTION+RESOLUTION/2]

if __name__ == '__main__':
    try:
        controller = MultiRobotController()
        while controller.node_waiting:
            print('waiting on the node')
            # rospy.sleep(0.1)
        for i in range(AGENT_NUMS):
            controller.agents_track[i]['odom_time'] = rospy.Time.now().to_sec()
        controller.publish_velocity_commands()
    except rospy.ROSInterruptException:
        pass
