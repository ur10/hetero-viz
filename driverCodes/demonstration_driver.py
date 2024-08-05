#!/usr/bin/env python3

import roslaunch
import rospy
import numpy as np
import rospkg
from task_env import TaskEnv
import pickle
import os

rospack = rospkg.RosPack()
PATH = rospack.get_path('marmot')
rospy.init_node('driverNode')
testSet = rospy.get_param('/testSet_name')
env = pickle.load(open(f'{testSet}/env_1/baseline.pkl', 'rb'))
Scale_factor = int(rospy.get_param('arena_scale'))

# Scale_factor = 10

# env = pickle.load(open(f'{testSet}/env_1/baseline.pkl', 'rb'))
# Scale_factor = 10

def get_matrix(dictionary, key):
    """
    :param key: the key to index
    :param dictionary: the dictionary for key to index
    """
    key_matrix = []
    for value in dictionary.values():
        key_matrix.append(value[key])
    return key_matrix

for i in range(0,3):
    env['agent'][i] = env['agent'][i+3]
    del env['agent'][i+3]
# self.env = env

START = []
for i in range(len(env['agent'])):
    task_dict = {'current_agent_num': 0, 'required_agent_num': max(env['tasks'][i]['requirements']),
                 'task_time': env['tasks'][i]['time']}
    # self.task_track.append(task_dict)
    env['agent'][i]['depot'][0] = -env['agent'][i]['depot'][0]
    START.append(env['agent'][i]['depot'] * Scale_factor + 0.1 * i)
GOALS = get_matrix(env['tasks'], 'location')
GOALS = (np.array(GOALS) * Scale_factor).tolist()
VIRTUAL_ROBOT_COUNT = 3
TASK_COUNT = len(env['tasks'])
REAL_ROBOT_MAP = {}


LEFT= [0, 0]
RESOLUTION = 0.5
COLOR = ['Orange','Cyan','Blue','Purple','Green','Yellow','Magenta','White','Black','Brown','Pink','Turquoise','Gold']
REAL_ROBOT_START = START[:]
REAL_ROBOT_GOAL = GOALS[:]

START = START[:VIRTUAL_ROBOT_COUNT]
ORIGINAL_GOALS = GOALS
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
        start = START[i]
        end = getCoord(GOALS[i])
        print(f' The start position is {start}')
        # Add the launch file for that robot in the list and prepare for launch
        cli_args2 = [PATH+'/launch/oneSpoofSet.launch', 'color:='+COLOR[(i+3)%len(COLOR)],'robot_name:='+str(i), 'pose_x:='+str(start[0]), 'pose_y:='+str(start[1]),'goal_x:='+str(end[0]),'goal_y:='+str(end[1])]
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

    for i in range(TASK_COUNT):
        start = ORIGINAL_GOALS[i]
        print(f' The start position is {start}')
        # Add the launch file for that robot in the list and prepare for launch
        cli_args3 = [PATH+'/launch/oneSpoofGoal.launch', 'task_color:='+COLOR[i%len(COLOR)],'task_id:='+str(i), 'goalX:='+str(start[0]), 'goalY:='+str(start[1])]
        roslaunch_file3 = roslaunch.rlutil.resolve_launch_arguments(cli_args3)[0]
        roslaunch_args3 = cli_args3[1:]
        launch_files.append([(roslaunch_file3, roslaunch_args3)])

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