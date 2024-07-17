#!/usr/bin/env python3  

import rospy
import os
import paramiko

rospy.init_node("autoSSH")

robotIP = rospy.get_param('robotSSHnode/IP')


ssh = paramiko.SSHClient()

ssh.load_system_host_keys()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

ssh.connect(robotIP, username="bingda", password="bingda")

ssh.exec_command("export ROS_PACKAGE_PATH:=/home/bingda/catkin_ws/src:/opt/ros/melodic/share:/home/bingda/td_ws/src")
ssh.exec_command("roslaunch base_control base_startup.launch")

# ssh.exec_command("source /opt/ros/melodic/setup.bash")


rospy.spin()

ssh.exec_command("rosnode kill --all")
ssh.exec_command("killall -9 base_control.py")
ssh.close()