#!/usr/bin/env python3  
import rospy
from marmot.msg import ROS_Task
from enum import Enum
import numpy as np
import copy 
import rospkg

import json

rospack = rospkg.RosPack()
PATH = rospack.get_path('marmot')


rospy.init_node('central_marmot')

VIRTUAL_ROBOT_COUNT = rospy.get_param("VIRTUAL_ROBOT_COUNT", default=30)
REAL_ROBOT_COUNT = rospy.get_param("REAL_ROBOT_COUNT", default=0)

GRAPH = None
TOPIC = None

class Status(Enum):
    STAGED = 200
    ENQUEUED = 201
    DONE = 202


def cb(data):
    global GRAPH
    # rospy.loginfo(data.taskID)
    # if(GRAPH.taskDict[data.taskID].status.value < data.status):
        # GRAPH.changeStatus(data.taskID)
    GRAPH.taskDict[data.taskID].status = Status(data.status)
    for taskID_ in GRAPH.graph[data.taskID]:
        # print(taskID_)
        task_ = GRAPH.taskDict[taskID_]
        task_.preqs[data.taskID] = max(data.status, task_.preqs[data.taskID])
        if task_.status.value<Status.ENQUEUED.value and task_.__checkIfReady__():
            GRAPH.changeStatus(taskID_)

class Task:
    # __actionDict__ = {0:np.array([0,0]), 1:np.array([0,1]), 2:np.array([1,0]), 3:np.array([0,-1]), 4:np.array([-1,0])}
    __actionDict__ = {0:np.array([0,0]), 1:np.array([1,0]), 2:np.array([0,1]), 3:np.array([-1,0]), 4:np.array([0,-1])}

    def __init__(self, tid, rid, start, action, time) -> None:
        self.taskID = tid
        self.robotID = rid
        self.action = action
        self.startPos = np.array(start)
        self.goalPos = np.array(self.__actionDict__[action]+start)
        self.time = time
        self.preqs = dict()
        self.status = Status.STAGED
        
    def __checkIfReady__(self):
        # print("Check if ready", self.preqs)
        if self.taskID-1 not in self.preqs:
            temp = list(self.preqs.values())
            if(np.array_equal(temp, np.full_like(temp, Status.DONE.value))):
                return True
            else:
                return False
        if self.preqs[self.taskID-1]>=Status.ENQUEUED.value:
            temp = copy.deepcopy(self.preqs)
            del temp[self.taskID-1]
            temp = list(temp.values())
            if (np.array_equal(temp, np.full_like(temp, Status.DONE.value))):
                return True
            else:
                return False
        return False
    
    def __publishTask__(self):
        global TOPIC
        message = ROS_Task()
        message.taskID = self.taskID
        message.robotID = self.robotID
        message.action = self.action
        message.status = self.status.value
        # print(message)
        # rospy.loginfo(message)
        TOPIC.publish(message)

    
    def __repr__(self) -> str:
        asd = "\n"
        for i in self.__dir__():
            if not i.startswith('__'):
                asd+=i
                asd+=": "
                asd+=str(getattr(self,i))
                asd+=", "

        return asd

class Graph:
    def __init__(self, taskList, startCells) -> None:
        print("Using original Swen Graph")
        self.currentPositions = copy.deepcopy(startCells)
        self.graph = dict()
        self.taskDict = dict()
        tId = 1
        self.robotList = []
        for rid in range(len(startCells)):
        
            prevTask = None
            for i, task in enumerate(taskList[:,rid]):
                t = Task(tId, rid, self.currentPositions[rid], task, i)
                # print(t)
                self.taskDict[tId] = t
                tId+=1
                self.currentPositions[rid] = t.goalPos
                self.addVertex(t)

                if(prevTask is None):
                    self.robotList.append(t)
                else:
                    self.addEdge(prevTask, t)

                prevTask = t
        # print(taskDict)
        for rid in range(len(startCells)):
            firstTid = self.robotList[rid].taskID
            for taskID in range(firstTid, firstTid+len(taskList)):
                if(taskID not in self.taskDict):
                    continue
                task = self.taskDict[taskID]
                
                for rid_ in range(len(startCells)):
                    if(rid != rid_):
                        # print(rid, rid_)
                        firstTid_ = self.robotList[rid_].taskID
                        for taskID_ in range(firstTid_, firstTid_+len(taskList)):
                            if(taskID_ not in self.taskDict):
                                continue
                            task_ = self.taskDict[taskID_]
                            if np.array_equal(task.startPos, task_.goalPos) and task.time<=task_.time:
                                # print(task, task_)
                                self.addEdge(task, task_)
                                break

        rospy.loginfo("Graph init Done")
    
    def addVertex(self, task):
        assert type(task) == Task
        if(task.taskID not in self.graph):
            self.graph[task.taskID] = list()

    def addEdge(self, taskFrom, taskTo):
        assert  type(taskFrom)==Task and taskFrom.taskID in self.graph
        assert type(taskTo)==Task and taskTo.taskID in self.graph
        taskTo.preqs[taskFrom.taskID] = taskFrom.status.value
        self.graph[taskFrom.taskID].append(taskTo.taskID)

    def changeStatus(self, taskID):
        # rospy.loginfo(taskID)
        task = self.taskDict[taskID]
        # print(taskID, task)
        task.status = Status(task.status.value+1)

        task.__publishTask__()

            
if __name__ == '__main__':


    TOPIC = rospy.Publisher("/comms",ROS_Task, queue_size=0)
    rospy.Subscriber("/comms", ROS_Task, cb)

    rospy.sleep(0.2)
    rospy.loginfo("Init Done Central Marmot")


    with open("/home/mapf/td_ws/src/Tanishq_MAPF/database/tasklist-30-10.txt") as f:
        t = json.load(f)

    with open("/home/mapf/td_ws/src/Tanishq_MAPF/database/graph-30-10.txt") as f:
        g = json.load(f)


    GRAPH = Graph([],[])

    GRAPH.robotList = {}

    for i in g:
        taskID = int(i)
        robotID = t[str(taskID)]['robotID']

        GRAPH.graph[taskID] = g[i]
        t_ = Task(taskID, robotID,[-1,-1],t[str(taskID)]['action'],-1)
        for pre in t[str(taskID)]['preqs']:
            t_.preqs[pre] = Status.STAGED.value

        if robotID not in GRAPH.robotList:
            GRAPH.robotList[robotID] = t_

        GRAPH.taskDict[taskID] = t_

    for i in GRAPH.robotList.values():
        if len(i.preqs)==0:
            # print(i)
            GRAPH.changeStatus(i.taskID)
    
    print("Compare DONE")


    rospy.spin()
        


    
