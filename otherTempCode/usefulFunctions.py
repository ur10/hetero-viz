import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.colors import hsv_to_rgb
import cv2
import math
import imageio
from enum import Enum
import copy

def getCenter(img, data):
    center = -np.asarray(data['origin'][:2])/data['resolution']
    print(center)

    center = -np.floor(np.asarray(data['origin'][:2])/data['resolution']).astype('int32')
    print(center)

    center = img.shape[1]-center[1], center[0]

    center = tuple(center)
    # Radius = 2
    # for i in range(-Radius,Radius+1):
    #     for j in range(-Radius, Radius+1):
    #         img[), (center[0]+j)] = 128
    return center

def getCoordinatesFromCell(cell, img, data):
    cell = np.array([cell[1], img.shape[0]-cell[0]])
    coord = cell*data['resolution']+np.array(data['origin'][:2]) - np.array([data['resolution']/2, data['resolution']/2])
    coord = np.round(coord, 2)
    return coord

def transformCell(cell = [13, 20],left = [930,920], scale = 8):
    _left = np.array(left)
    _cell = np.array(cell)
    # print(_left, _cell*scale)
    return _left+(_cell*scale)


def init_colors(n):
    """the colors of agents and goals"""
    c = {a + 1: hsv_to_rgb(np.array([a / float(n), 1, 1])) for a in range(n)}
    c[0] = [1,1,1]
    c[-1] = [0,0,0]
    c[-2] = [0.5,0.5,0.5]
    return c


def getRectPoints(coord, scale):
    base = [coord[1]*scale, coord[0]*scale]
    return np.array([base, [base[0]+scale-1, base[1]], [base[0]+scale-1,base[1]+scale-1], [base[0], base[1]+scale-1]])    

def pixelForText(coord, scale):
    base = [coord[1]*scale, coord[0]*scale]
    return [int(math.floor(base[0]+scale*1/4)), int(math.floor(base[1]+scale*3/4))]


def getCenter(coord, scale):
    base = [coord[1]*scale, coord[0]*scale]
    return [int(math.floor(base[0]+scale/2)), int(math.floor(base[1]+scale/2))]

def getTriPoints( coord, scale):
    base = [coord[1]*scale, coord[0]*scale]
    return  np.array([[int(math.floor(base[0]+scale/2)), base[1]], [base[0]+scale-1,base[1]+scale-1], [base[0], base[1]+scale-1]])    

def renderWorld(scale=20, world = np.zeros(1),agents=[], goals=[]):
    size = world.shape
    numAgents = len(agents)
        
    screen_height = scale*size[0]
    screen_width = scale*size[1]

    colours = init_colors(numAgents)

    scene = np.zeros([screen_height, screen_width, 3])

    for coord,val in np.ndenumerate(world):
        cv2.fillPoly(scene, pts=[getRectPoints(coord=coord, scale=scale)], color=colours[val])

    for val,coord in enumerate(goals):
        cv2.circle(scene, getCenter(coord=coord, scale=scale), math.floor(scale/2)-1, colours[val+1], -1)
        cv2.putText(scene, str(val+1), pixelForText(coord, scale), cv2.FONT_HERSHEY_SIMPLEX,scale/(40*(int(np.log10(val+1))+1)), (0,0,0), int(scale/20))


    for val,coord in enumerate(agents):
        cv2.fillPoly(scene, pts=[getRectPoints(coord=coord, scale=scale)], color=colours[val+1])
        cv2.putText(scene, str(val+1), pixelForText(coord, scale), cv2.FONT_HERSHEY_SIMPLEX,scale/(40*(int(np.log10(val+1))+1)), (0,0,0), int(scale/20))

    scene = scene*255
    scene = scene.astype(dtype='uint8')
    return scene


def returnAsType(arr, type):
    if(type=='np'): # numpy array
        return np.array(arr)
        
    elif(type=='mat'): # to be used directly as a cell of matrix
        return tuple(arr)
    else:
        raise Exception("Invalid Type as input")

class Agent():
    dirDict = {0: (0, 0), 1: (0, 1), 2: (1, 0), 3: (0, -1), 
               4: (-1, 0), 5: (1, 1), 6: (1, -1), 7: (-1, -1), 8: (-1, 1)}  # x,y operation for corresponding action
    
    oppositeAction = {0:0, 1:3, 2:4, 3:1, 4:2}

    def __init__(self, world):
        self.__position = np.array([-1,-1])
        self.__goal = np.array([-1,-1])

    def setPos(self, pos):
        self.__position = np.array(pos)

    def getPos(self, type='np'):
        return returnAsType(self.__position, type)

    def setGoal(self, goal):

        self.__goal = np.array(goal)

    def getGoal(self, type='np'):
        return returnAsType(self.__goal, type)
    
    def takeStep(self, action):
        step = np.array(self.dirDict[action])
        self.setPos(np.add(self.getPos(), step))
        self.previousAction = self.oppositeAction[action]
        

class MapfGym():
    def __init__(self, world, start, end):
        self.obstacleMap = world
        self.agentList = [Agent(self.obstacleMap) for i in range(len(start))]
        for idx, agent in enumerate(self.agentList):
            agent.setPos(start[idx])
            agent.setGoal(end[idx])

    def jointStep(self, actions):
        for agentIdx, agent in enumerate(self.agentList):
            agent.takeStep(actions[agentIdx])

    def _render(self):
        goals = []
        agents = []
        for i in self.agentList:
            agents.append(i.getPos('mat'))
            goals.append(i.getGoal('mat'))

        return renderWorld(world=self.obstacleMap, agents=agents,goals=goals)
    

def make_gif(images, file_name):
    """record gif"""
    imageio.mimwrite(file_name, images, subrectangles=True)
    print("wrote gif")

class Status(Enum):
    STAGED = 200
    ENQUEUED = 201
    DONE = 202


class Task:
    __actionDict__ = {0:np.array([0,0]), 1:np.array([0,1]), 2:np.array([1,0]), 3:np.array([0,-1]), 4:np.array([-1,0])}
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
        self.currentPositions = copy.deepcopy(startCells)
        self.graph = dict()
        self.taskDict = dict()
        tId = 1
        self.robotList = []
        for rid in range(len(startCells)):
        
            prevTask = None
            for i, task in enumerate(taskList[:,rid]):
                if(task==0):
                    continue
                # print(tId, rid, currentPositions[rid], task, i)
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
                    if(rid == rid_):
                        continue
                    else:
                        firstTid_ = self.robotList[rid_].taskID
                        for taskID_ in range(firstTid_, firstTid_+len(taskList)):
                            if(taskID_ not in self.taskDict):
                                continue
                            task_ = self.taskDict[taskID_]
                            if np.array_equal(task.startPos, task_.goalPos) and task.time<=task_.time:
                                self.addEdge(task, task_)
                                break
        # rospy.loginfo("Graph Init Done")
            
    
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
            