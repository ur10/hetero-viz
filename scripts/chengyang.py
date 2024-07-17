#!/usr/bin/env python3  
import rospy
from marmot.msg import ROS_SVO_Task
from enum import Enum
import numpy as np
import copy 
import rospkg

import pickle

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
    END = -3


def cb(data):
    global pubList, GRAPH

    # print("New Data", data)
    if GRAPH is not None:
        if(data.status==Status.END.value):
            return
        
        if(data.taskID == -1):
            if(GRAPH.robotList[data.robotID].status==Status.ENQUEUED):
                GRAPH.robotList[data.robotID].__publishTask__()
        else:
            GRAPH.taskDict[data.taskID].status = Status(max(data.status, GRAPH.taskDict[data.taskID].status.value))
            if (GRAPH.taskDict[data.taskID].status.value == Status.DONE.value) and data.taskID not in pubList:
                # print("adding to list", data.taskID, data.status, GRAPH.taskDict[data.taskID].status.value)
                pubList.append(data.taskID)


class Task:
    # __actionDict__ = {0:np.array([0,0]), 1:np.array([0,1]), 2:np.array([1,0]), 3:np.array([0,-1]), 4:np.array([-1,0])}
    __actionDict__ = {0:np.array([0,0]), 1:np.array([1,0]), 2:np.array([0,1]), 3:np.array([-1,0]), 4:np.array([0,-1])}

    def __init__(self, tid, rid, start, action, time, neigh, svo_avg, svo_distri) -> None:
        self.taskID = tid
        self.robotID = rid
        self.action = action
        self.startPos = np.array(start)
        self.goalPos = np.array(self.__actionDict__[action]+start)
        self.time = time
        self.neigh = neigh-1
        self.svo_avg = svo_avg
        self.svo_distri = svo_distri
        self.preqs = dict()
        self.status = Status.STAGED
    
    def __publishTask__(self):
        global TOPIC
        message = ROS_SVO_Task()
        message.taskID = int(self.taskID)
        message.robotID = int(self.robotID)
        message.action = int(self.action)
        message.status = self.status.value
        message.neighbour = int(self.neigh)
        message.svo_avg = self.svo_avg
        message.svo_distri = self.svo_distri

        # print(message)
        # rospy.loginfo(message)
        TOPIC[self.robotID].publish(message)

    
    def __repr__(self) -> str:
        asd = "\n"
        for i in self.__dir__():
            if not i.startswith('__'):
                asd+=i
                asd+=": "
                asd+=str(getattr(self,i))
                asd+=", "

        return asd

class GraphTD:
    def __init__(self, taskList, startCells, neigh, svo_avg, svo_distri) -> None:
        positions = {}
        self.currentPositions = copy.deepcopy(startCells)

        self.graph = dict()
        self.taskDict = dict()
        tId = 1
        self.robotList = []
        for rid in range(len(startCells)):
        
            prevTask = None
            for i, task in enumerate(taskList[:,rid]):

                # print(tId, rid, currentPositions[rid], task, i)
                t = Task(tId, rid, self.currentPositions[rid], task, i,neigh[i,rid], svo_avg[i,rid], svo_distri[i,rid,:])
                # print(t)
                self.taskDict[tId] = t
                tId+=1
                self.currentPositions[rid] = t.goalPos
                self.addVertex(t)

                if(tuple(t.goalPos) not in positions):
                    positions[tuple(t.goalPos)] = dict()
                positions[tuple(t.goalPos)][t.time] = t.taskID

                if(prevTask is None):
                    self.robotList.append(t)
                else:
                    self.addEdge(prevTask, t)

                prevTask = t
        # print(self.taskDict)

        # self.pos = positions
        for pos in positions:
            timeList = sorted(positions[pos].keys())
            # print(timeList)
            # print(positions[pos][0])
            for i in range(len(timeList)):
                for j in range(i+1, len(timeList)):
                    # print(positions[pos], positions[pos][timeList[i]], positions[pos][timeList[j]])
                    self.addEdge(self.taskDict[positions[pos][timeList[i]]+1], self.taskDict[positions[pos][timeList[j]]])

        for t in range(1, len(self.taskDict), len(taskList)):
            # print(self.taskDict[t])
            if(tuple(self.taskDict[t].startPos) in positions):
                for time in positions[tuple(self.taskDict[t].startPos)]:
                    self.addEdge(self.taskDict[t], self.taskDict[positions[tuple(self.taskDict[t].startPos)][time]])



        for rid in range(len(startCells)):
            firstTid = self.robotList[rid].taskID
            for taskID in range(firstTid, firstTid+len(taskList)):
                if(taskID not in self.taskDict):
                    continue
                task = self.taskDict[taskID]
            
    
    def addVertex(self, task):
        assert type(task) == Task
        if(task.taskID not in self.graph):
            self.graph[task.taskID] = list()


    def addEdge(self, taskFrom, taskTo):
        assert  type(taskFrom)==Task and taskFrom.taskID in self.graph
        assert type(taskTo)==Task and taskTo.taskID in self.graph

        if(taskFrom.taskID in taskTo.preqs or taskFrom.taskID==taskTo.taskID):
            return
        for t in self.graph[taskFrom.taskID]:
            if(self.taskDict[t].robotID==taskTo.robotID):
                return


        taskTo.preqs[taskFrom.taskID] = taskFrom.status.value
        self.graph[taskFrom.taskID].append(taskTo.taskID)

    def changeStatus(self, taskID):
        # rospy.loginfo(taskID)
        task = self.taskDict[taskID]
        # print(taskID, task)
        task.status = Status(task.status.value+1)

        task.__publishTask__()

            
if __name__ == '__main__':

    actionList= np.loadtxt(PATH+'/otherTempCode/actions.txt')
    actionList = actionList.astype(np.int16)
    START=[(2, 7),(20, 1),(17, 16),(2, 0),(10, 1),(16, 22),(17, 7),(1, 4),(5, 5),(17, 18),(7, 22),(21, 20),(7, 1),(1, 2),(15, 21),(2, 17),(5, 10),(17, 13),(10, 22),(2, 21),(8, 4),(8, 19),(5, 3),(20, 3),(14, 12),(19, 20),(1, 7),(2, 14),(14, 21),(15, 2),(11, 21),(17, 1),(2, 10),(5, 9),(1, 5),(13, 1),(22, 2),(15, 14),(3, 21),(2, 8),(1, 0),(22, 16),(0, 14),(21, 2),(8, 22),(18, 8),(18, 1),(0, 20),(1, 22),(17, 5),(11, 8),(17, 11),(8, 17),(0, 9),(16, 0),(17, 2),(4, 8),(14, 19),(2, 11),(12, 14),(8, 6),(21, 1),(2, 16),(21, 17),(0, 18),(19, 22),(20, 16),(9, 2),(0, 17),(7, 8),(1, 6),(11, 0),(17, 3),(0, 0),(9, 14),(22, 10),(8, 9),(21, 16),(21, 15),(1, 1),(20, 15),(19, 1),(9, 22),(22, 3),(2, 9),(7, 21),(14, 13),(0, 21),(12, 8),(0, 2),(18, 20),(17, 22),(1, 10),(0, 10),(5, 4),(22, 5),(13, 8),(2, 22),(5, 15),(6, 14),(1, 13),(5, 19),(22, 7),(0, 12),(15, 22),(21, 19),(5, 20),(10, 8),(21, 7),(22, 0),(7, 2),(1, 16),(11, 19),(14, 8),(8, 5),(1, 19),(1, 11),(0, 5),(1, 20),(8, 7),(5, 18),(14, 6),(22, 21),(14, 17),(6, 2),(18, 14),(15, 20),(12, 22),(21, 12),(6, 8),(0, 7),(11, 17),(22, 20),(0, 11),(11, 6),(14, 2),(2, 3),(16, 2),(14, 10),(0, 13),(16, 1),(15, 1),(21, 21),(18, 21),(0, 4),(5, 22),(20, 6),(1, 9),(20, 14),(12, 0),(0, 8),(20, 5),(20, 13),(12, 2),(13, 22),(1, 3),(21, 10),(11, 3),(4, 1),(17, 21),(20, 18),(14, 3),(14, 14),(0, 6),(7, 14),(22, 8),(8, 1),(2, 19),(12, 1),(20, 4),(18, 0),(1, 17),(5, 1),(22, 18),(19, 0),(22, 9),(9, 8),(17, 17),(2, 12),(21, 8),(8, 21),(20, 19),(5, 11),(17, 10),(17, 6),(22, 14),(3, 20),(11, 15),(3, 2),(8, 3),(17, 15),(13, 21),(18, 22),(14, 1),(21, 3),(6, 0),(8, 8),(14, 11),(21, 13),(11, 1),(10, 20),(22, 13),(5, 13),(6, 21),(0, 22),(3, 22),(0, 16),(3, 14),(4, 14),(1, 15),(5, 14),(8, 15)]
    GOALS = [(9, 0),(0, 19),(11, 2),(0, 6),(5, 0),(11, 4),(8, 6),(14, 21),(22, 3),(11, 14),(20, 14),(0, 5),(15, 8),(17, 12),(0, 0),(19, 14),(1, 22),(0, 11),(3, 8),(22, 15),(5, 22),(21, 8),(17, 11),(3, 21),(5, 12),(14, 1),(17, 6),(5, 4),(1, 19),(17, 18),(14, 13),(17, 0),(2, 21),(11, 11),(4, 21),(5, 13),(11, 5),(11, 9),(8, 4),(20, 2),(16, 2),(17, 16),(5, 17),(0, 8),(8, 16),(6, 1),(14, 20),(22, 6),(22, 1),(4, 22),(5, 14),(6, 8),(20, 8),(11, 16),(22, 13),(0, 22),(11, 20),(20, 18),(17, 5),(10, 20),(22, 22),(22, 0),(8, 8),(5, 10),(21, 16),(22, 12),(17, 9),(12, 1),(21, 5),(14, 2),(22, 2),(13, 0),(8, 18),(21, 6),(21, 0),(20, 9),(10, 22),(21, 1),(20, 22),(10, 1),(12, 8),(7, 8),(0, 10),(13, 21),(1, 3),(8, 13),(22, 17),(21, 2),(18, 8),(11, 8),(17, 2),(18, 1),(19, 21),(19, 22),(5, 5),(15, 0),(10, 21),(16, 20),(5, 21),(8, 5),(13, 8),(0, 15),(22, 8),(13, 20),(21, 3),(2, 12),(22, 20),(5, 19),(10, 8),(4, 0),(1, 9),(16, 22),(2, 0),(5, 1),(5, 16),(15, 20),(0, 12),(0, 1),(21, 21),(14, 18),(2, 1),(14, 4),(14, 5),(1, 17),(17, 13),(6, 14),(18, 0),(17, 3),(10, 2),(2, 13),(22, 21),(4, 1),(1, 20),(17, 21),(0, 17),(20, 6),(1, 2),(4, 8),(14, 9),(14, 10),(5, 20),(14, 11),(14, 22),(3, 22),(22, 9),(12, 2),(11, 1),(22, 7),(4, 14),(22, 14),(0, 13),(17, 22),(0, 3),(20, 21),(2, 7),(1, 8),(21, 14),(11, 13),(10, 0),(15, 1),(11, 19),(22, 11),(11, 18),(6, 2),(19, 1),(21, 4),(0, 4),(3, 2),(18, 20),(5, 6),(15, 21),(20, 0),(1, 14),(2, 15),(21, 19),(20, 11),(11, 22),(17, 19),(16, 0),(22, 10),(4, 20),(13, 1),(20, 16),(22, 4),(8, 12),(9, 21),(8, 1),(7, 14),(12, 21),(17, 7),(8, 2),(2, 4),(14, 8),(20, 1),(21, 11),(3, 1),(18, 2),(9, 8),(5, 7),(21, 17),(8, 0),(0, 18),(8, 20),(2, 14),(8, 14),(1, 21),(0, 14),(1, 13),(2, 19),(1, 16),(6, 21),(1, 18)]
    
    
    REAL_ROBOT_START = START[202:]
    START = START[:VIRTUAL_ROBOT_COUNT]
    START+=REAL_ROBOT_START[:REAL_ROBOT_COUNT]

    GOALS = GOALS[:VIRTUAL_ROBOT_COUNT]

    size = actionList.shape

    print("Total robots", REAL_ROBOT_COUNT+VIRTUAL_ROBOT_COUNT)

    taskList = np.zeros((size[0], REAL_ROBOT_COUNT+VIRTUAL_ROBOT_COUNT),dtype=np.int16)

    for i in range(REAL_ROBOT_COUNT+VIRTUAL_ROBOT_COUNT):
        if i>=VIRTUAL_ROBOT_COUNT:
            taskList[:,i] = actionList[:,202+i-VIRTUAL_ROBOT_COUNT]
        else:
            taskList[:,i] = actionList[:,i]


    TOPIC = {}

    for i in range(VIRTUAL_ROBOT_COUNT+REAL_ROBOT_COUNT):
        TOPIC[i] = rospy.Publisher("/comms_nexus"+str(i),ROS_SVO_Task, queue_size=0)
        rospy.Subscriber("/comms_nexus"+str(i), ROS_SVO_Task, cb,queue_size=10000)

    5

    rospy.sleep(0.2)
    rospy.loginfo("Init Done Central Marmot")



    GRAPH = GraphTD(taskList,START)

    pubList = list()
    for i in GRAPH.robotList:
        if len(i.preqs)==0:
            # pubList.append(i.taskID)
            GRAPH.taskDict[i.taskID].status = Status.ENQUEUED
            GRAPH.taskDict[i.taskID].__publishTask__()

        # else:
            # print("Robot Preqs",i.robotID, i.taskID, i.preqs)

    while not rospy.is_shutdown():
        while len(pubList)!=0:
            taskID = pubList.pop(0)
            task = GRAPH.taskDict[taskID]

            # print("Check for", taskID, task.status.value, GRAPH.graph[taskID])
            assert task.status.value == Status.DONE.value

            if len(GRAPH.graph[taskID])==0:
                message = ROS_SVO_Task()
                message.status = Status.END.value
                TOPIC[GRAPH.taskDict[taskID].robotID].publish(message)
            
            for taskID_ in GRAPH.graph[taskID]:
                # print(taskID_)
                task_ = GRAPH.taskDict[taskID_]
                # for i in task_.preqs:
                #     task_.preqs[i] = GRAPH.taskDict[i].status
                if(task_.status==Status.STAGED):
                    changeStatus = True
                    for i in task_.preqs:
                        
                        if GRAPH.taskDict[i].robotID==task_.robotID:
                            if(GRAPH.taskDict[i].status.value == Status.STAGED.value):
                                changeStatus = False
                                break
                        else:
                            if(GRAPH.taskDict[i].status.value<Status.DONE.value):
                                changeStatus = False
                                break

                    # print(taskID_, changeStatus, GRAPH.taskDict[taskID_].preqs)
                    if(changeStatus):
                        GRAPH.taskDict[taskID_].status = Status.ENQUEUED
                        GRAPH.taskDict[taskID_].__publishTask__()
                elif task_.status==Status.ENQUEUED:
                    GRAPH.taskDict[taskID_].__publishTask__()



        rospy.sleep(0.1)


    print("Compare DONE")


    rospy.spin()
        


    
