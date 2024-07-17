#!/usr/bin/env python3

import roslaunch
import rospy
import numpy as np
import rospkg
rospack = rospkg.RosPack()
PATH = rospack.get_path('marmot')

ROBOT_COUNT=8
LEFT= [-8, 5.6]
RESOLUTION = 0.4
COLOR = ['Red','Orange','Green','Green','Cyan','Blue','Purple','Magenta','Yellow','White','Black','Gray','Brown','Pink','Turquoise','Gold','Silver']
START=[(7, 12), (4, 34), (13, 6), (10, 23), (12, 30), (4, 28), (25, 34), (10, 21)]
GOALS = [(11, 29), (10, 37), (6, 16), (9, 34), (10, 14), (14, 2), (14, 35), (15, 4)]

def getCoord(i):
    return [LEFT[0]+i[1]*RESOLUTION, LEFT[1]-i[0]*RESOLUTION]

rospy.init_node('driverNode')

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
    for i in range (ROBOT_COUNT):
        for j in temp:
            k = j.replace("nexus0", 'nexus'+str(i))
            w.writelines(k)
    
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
    cli_args1 = ['marmot', 'empty_world.launch']
    roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)
    launch_files = [roslaunch_file1]
    roslaunch.parent.ROSLaunchParent(uuid, launch_files[0]).start()

    rospy.set_param('/robotCount', ROBOT_COUNT)

    
    for i in range (ROBOT_COUNT): # For each robot of that type (given by the count in SIMULATION.yaml)

        # Get coordinates for the start node of the robot
        start = getCoord(START[i])
        end = getCoord(GOALS[i])

        # Add the launch file for that robot in the list and prepare for launch
        cli_args2 = [PATH+'/launch/oneSet.launch', 'color:='+COLOR[i],'robot_name:='+str(i), 'pose_x:='+str(start[0]), 'pose_y:='+str(start[1]),'goal_x:='+str(end[0]),'goal_y:='+str(end[1])]
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
