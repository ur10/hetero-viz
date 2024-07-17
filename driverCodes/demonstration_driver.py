#!/usr/bin/env python3

import roslaunch
import rospy
import numpy as np
import rospkg
from task_env import TaskEnv
import pickle

rospack = rospkg.RosPack()
PATH = rospack.get_path('marmot')
rospy.init_node('driverNode')
testSet = "/home/ur10/mapf_ws/testSet_simulation"

env = pickle.load(open(f'{testSet}/env_0/RL.pkl', 'rb'))

START = []
for i in range(len(env['agent'])):
    task_dict = {'current_agent_num': 0, 'required_agent_num': max(env['tasks'][i]['requirements']),
                 'task_time': env['tasks'][i]['time']}
    # self.task_track.append(task_dict)
    START.append(env['agent'][i]['depot'] *5)
VIRTUAL_ROBOT_COUNT = 5
# REAL_ROBOT_MAP = { 2:"102", 9:"104",5:"112",8:"109",1:"106"}
# REAL_ROBOT_MAP = {1:"106",3:"108",  7:"113", 11:"110",9:"104", 6:"103", 4:"111", 8:"109", 5:"112", 2:"102"}
REAL_ROBOT_MAP = {}


LEFT= [0, 0]
RESOLUTION = 0.5
COLOR = ['Orange','Green','Cyan','Blue','Purple','Magenta','Yellow','White','Black','Brown','Pink','Turquoise','Gold']
# START=[(1, 1),(2, 2),(3, 3),(4, 4),(5, 5),(6, 6),(7, 7),(1, 4),(5, 5),(17, 18),(7, 22),(21, 20),(7, 1),(1, 2),(15, 21),(2, 17),(5, 10),(17, 13),(10, 22),(2, 21),(8, 4),(8, 19),(5, 3),(20, 3),(14, 12),(19, 20),(1, 7),(2, 14),(14, 21),(15, 2),(11, 21),(17, 1),(2, 10),(5, 9),(1, 5),(13, 1),(22, 2),(15, 14),(3, 21),(2, 8),(1, 0),(22, 16),(0, 14),(21, 2),(8, 22),(18, 8),(18, 1),(0, 20),(1, 22),(17, 5),(11, 8),(17, 11),(8, 17),(0, 9),(16, 0),(17, 2),(4, 8),(14, 19),(2, 11),(12, 14),(8, 6),(21, 1),(2, 16),(21, 17),(0, 18),(19, 22),(20, 16),(9, 2),(0, 17),(7, 8),(1, 6),(11, 0),(17, 3),(0, 0),(9, 14),(22, 10),(8, 9),(21, 16),(21, 15),(1, 1),(20, 15),(19, 1),(9, 22),(22, 3),(2, 9),(7, 21),(14, 13),(0, 21),(12, 8),(0, 2),(18, 20),(17, 22),(1, 10),(0, 10),(5, 4),(22, 5),(13, 8),(2, 22),(5, 15),(6, 14),(1, 13),(5, 19),(22, 7),(0, 12),(15, 22),(21, 19),(5, 20),(10, 8),(21, 7),(22, 0),(7, 2),(1, 16),(11, 19),(14, 8),(8, 5),(1, 19),(1, 11),(0, 5),(1, 20),(8, 7),(5, 18),(14, 6),(22, 21),(14, 17),(6, 2),(18, 14),(15, 20),(12, 22),(21, 12),(6, 8),(0, 7),(11, 17),(22, 20),(0, 11),(11, 6),(14, 2),(2, 3),(16, 2),(14, 10),(0, 13),(16, 1),(15, 1),(21, 21),(18, 21),(0, 4),(5, 22),(20, 6),(1, 9),(20, 14),(12, 0),(0, 8),(20, 5),(20, 13),(12, 2),(13, 22),(1, 3),(21, 10),(11, 3),(4, 1),(17, 21),(20, 18),(14, 3),(14, 14),(0, 6),(7, 14),(22, 8),(8, 1),(2, 19),(12, 1),(20, 4),(18, 0),(1, 17),(5, 1),(22, 18),(19, 0),(22, 9),(9, 8),(17, 17),(2, 12),(21, 8),(8, 21),(20, 19),(5, 11),(17, 10),(17, 6),(22, 14),(3, 20),(11, 15),(3, 2),(8, 3),(17, 15),(13, 21),(18, 22),(14, 1),(21, 3),(6, 0),(8, 8),(14, 11),(21, 13),(11, 1),(10, 20),(22, 13),(5, 13),(6, 21),(0, 22),(3, 22),(0, 16),(3, 14),(4, 14),(1, 15),(5, 14),(8, 15)]
GOALS = [(9, 0),(0, 19),(11, 2),(0, 6),(5, 0),(11, 4),(8, 6),(14, 21),(22, 3),(11, 14),(20, 14),(0, 5),(15, 8),(17, 12),(0, 0),(19, 14),(1, 22),(0, 11),(3, 8),(22, 15),(5, 22),(21, 8),(17, 11),(3, 21),(5, 12),(14, 1),(17, 6),(5, 4),(1, 19),(17, 18),(14, 13),(17, 0),(2, 21),(11, 11),(4, 21),(5, 13),(11, 5),(11, 9),(8, 4),(20, 2),(16, 2),(17, 16),(5, 17),(0, 8),(8, 16),(6, 1),(14, 20),(22, 6),(22, 1),(4, 22),(5, 14),(6, 8),(20, 8),(11, 16),(22, 13),(0, 22),(11, 20),(20, 18),(17, 5),(10, 20),(22, 22),(22, 0),(8, 8),(5, 10),(21, 16),(22, 12),(17, 9),(12, 1),(21, 5),(14, 2),(22, 2),(13, 0),(8, 18),(21, 6),(21, 0),(20, 9),(10, 22),(21, 1),(20, 22),(10, 1),(12, 8),(7, 8),(0, 10),(13, 21),(1, 3),(8, 13),(22, 17),(21, 2),(18, 8),(11, 8),(17, 2),(18, 1),(19, 21),(19, 22),(5, 5),(15, 0),(10, 21),(16, 20),(5, 21),(8, 5),(13, 8),(0, 15),(22, 8),(13, 20),(21, 3),(2, 12),(22, 20),(5, 19),(10, 8),(4, 0),(1, 9),(16, 22),(2, 0),(5, 1),(5, 16),(15, 20),(0, 12),(0, 1),(21, 21),(14, 18),(2, 1),(14, 4),(14, 5),(1, 17),(17, 13),(6, 14),(18, 0),(17, 3),(10, 2),(2, 13),(22, 21),(4, 1),(1, 20),(17, 21),(0, 17),(20, 6),(1, 2),(4, 8),(14, 9),(14, 10),(5, 20),(14, 11),(14, 22),(3, 22),(22, 9),(12, 2),(11, 1),(22, 7),(4, 14),(22, 14),(0, 13),(17, 22),(0, 3),(20, 21),(2, 7),(1, 8),(21, 14),(11, 13),(10, 0),(15, 1),(11, 19),(22, 11),(11, 18),(6, 2),(19, 1),(21, 4),(0, 4),(3, 2),(18, 20),(5, 6),(15, 21),(20, 0),(1, 14),(2, 15),(21, 19),(20, 11),(11, 22),(17, 19),(16, 0),(22, 10),(4, 20),(13, 1),(20, 16),(22, 4),(8, 12),(9, 21),(8, 1),(7, 14),(12, 21),(17, 7),(8, 2),(2, 4),(14, 8),(20, 1),(21, 11),(3, 1),(18, 2),(9, 8),(5, 7),(21, 17),(8, 0),(0, 18),(8, 20),(2, 14),(8, 14),(1, 21),(0, 14),(1, 13),(2, 19),(1, 16),(6, 21),(1, 18)]

REAL_ROBOT_START = START[:]
REAL_ROBOT_GOAL = GOALS[:]

START = START[:VIRTUAL_ROBOT_COUNT]
GOALS = GOALS[:VIRTUAL_ROBOT_COUNT]

REAL_ROBOT_COUNT = len(list(REAL_ROBOT_MAP.keys()))
# REAL_ROBOT_COUNT = 10

rospy.set_param("VIRTUAL_ROBOT_COUNT", VIRTUAL_ROBOT_COUNT)
rospy.set_param("REAL_ROBOT_COUNT", REAL_ROBOT_COUNT)



def getCoord(i):
    return [LEFT[0]+i[1]*RESOLUTION+RESOLUTION/2, LEFT[1]+i[0]*RESOLUTION+RESOLUTION/2]


def rviz_write():
    # Part 1: Write a non changing piece for an rviz file
    read = open(PATH+"/rviz/part1", "r")
    w = open(PATH+"/rviz/dynamic_rviz.rviz","w")
    temp = read.readlines()

    w.writelines(temp)
    read.close()

    # Part 2: Read the part that we will change to visualize the number of robots we need
    read = open(PATH+"/rviz/agent", "r")
    temp = read.readlines()
    read.close()

    # Write for part 2
    for i in range(VIRTUAL_ROBOT_COUNT+REAL_ROBOT_COUNT):
        for j in temp:
            k = j.replace("nexus0", 'nexus'+str(i))
            w.writelines(k)

    # for i in range (REAL_ROBOT_COUNT):
    #     for j in temp:
    #         k = j.replace("nexus0", 'nexus'+str(VIRTUAL_ROBOT_COUNT+i))
    #         w.writelines(k)
    #
    # Part 3: Write some more constant lines for rviz files
    read = open(PATH+"/rviz/part2", "r")
    temp = read.readlines()
    w.writelines(temp)
    read.close()
    w.close()


def main():
    # Start an instance of ROSLAUNCH API 
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    rviz_write() #Generate the appropriate rviz file
    rospy.sleep(1)


    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    # roslaunch hospitalMinimal.launch
    ## this starts our barebone environment and initilizes a list which stores all files we need to launch
    cli_args1 = ['marmot', 'demonstration.launch']
    roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)
    launch_files = [roslaunch_file1]
    roslaunch.parent.ROSLaunchParent(uuid, launch_files[0]).start()

    for i in range (VIRTUAL_ROBOT_COUNT): # For each robot of that type (given by the count in SIMULATION.yaml)

        # Get coordinates for the start node of the robot
        start = getCoord(START[i])
        end = getCoord(GOALS[i])
        print(f' The start position is {start}')
        # Add the launch file for that robot in the list and prepare for launch
        cli_args2 = [PATH+'/launch/oneSpoofSet.launch', 'color:='+COLOR[i%len(COLOR)],'robot_name:='+str(i), 'pose_x:='+str(start[0]), 'pose_y:='+str(start[1]),'goal_x:='+str(end[0]),'goal_y:='+str(end[1])]
        roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)[0]
        roslaunch_args2 = cli_args2[1:]
        launch_files.append([(roslaunch_file2, roslaunch_args2)])

    for i in range (REAL_ROBOT_COUNT):
        # start = getCoord(REAL_ROBOT_START[i])
        # end = getCoord(REAL_ROBOT_GOAL[i])
        start = REAL_ROBOT_START[i]
        end = getCoord(REAL_ROBOT_GOAL[i])

        idx = VIRTUAL_ROBOT_COUNT+i
        # Add the launch file for that robot in the list and prepare for launch

        #SIM
        # cli_args2 = [PATH+'/launch/oneSpoofSet.launch', 'color:='+COLOR[idx%len(COLOR)],'robot_name:='+str(idx), 'pose_x:='+str(start[0]), 'pose_y:='+str(start[1]),'goal_x:='+str(end[0]),'goal_y:='+str(end[1])]

        #REAL
        robotRealID = list(REAL_ROBOT_MAP.keys())[i]
        # cli_args2= [PATH+'/launch/oneRealSet.launch', 'color:='+COLOR[idx%len(COLOR)],'robot_name:='+str(idx), 'robot_real_id:='+str(robotRealID),'robot_IP:=192.168.0.'+str(REAL_ROBOT_MAP[robotRealID]),'pose_x:='+str(start[0]), 'pose_y:='+str(start[1]),'goal_x:='+str(end[0]),'goal_y:='+str(end[1])]
        cli_args2= [PATH+'/launch/oneRealSet.launch', 'color:=Red','robot_name:='+str(idx), 'robot_real_id:='+str(robotRealID),'robot_IP:=192.168.0.'+str(REAL_ROBOT_MAP[robotRealID]),'pose_x:='+str(start[0]), 'pose_y:='+str(start[1]),'goal_x:='+str(end[0]),'goal_y:='+str(end[1])]

        roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)[0]
        roslaunch_args2 = cli_args2[1:]
        launch_files.append([(roslaunch_file2, roslaunch_args2)])

    # Finally launch all the commands we have in the list
    parent = {}
    for id, val in enumerate(launch_files[1:]):
        print(val)
        parent[id] = roslaunch.parent.ROSLaunchParent(uuid, val)
        parent[id].start()
    

    # node = roslaunch.core.Node(package="marmot", node_type="centralMarmot2.py", name="central_marmot", namespace='/', output="screen")
    # launch.launch(node)
    



    # If this node closes, everything shuts down, so we let it sleep instead
    rospy.spin() 


if __name__=='__main__':
    main()
