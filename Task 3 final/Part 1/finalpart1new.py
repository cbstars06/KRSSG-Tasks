import math
from pickle import FALSE, TRUE
import random
import cv2 as cv
import numpy as np




img = cv.imread('Task 3/task3.1.png')
imgHSV = cv.cvtColor(img,cv.COLOR_BGR2HSV)

#Obstacle Image
lower = np.array([0,0,47])
higher = np.array([177,0,255])
mask_obs = cv.inRange(imgHSV,lower,higher)
imgObstacle = cv.bitwise_and(img,img,mask=mask_obs)

#Start Image
lower1 = np.array([8,0,0])
higher1 = np.array([100,255,255])
mask_start = cv.inRange(imgHSV,lower1,higher1)
imgStart = cv.bitwise_and(img,img,mask=mask_start)

#Goal Image
lower_red = np.array([0,50,50])
upper_red = np.array([10,255,255])
mask0 = cv.inRange(imgHSV, lower_red, upper_red)
# upper mask (170-180)
lower_red = np.array([170,50,50])
upper_red = np.array([180,255,255])
mask1 = cv.inRange(imgHSV, lower_red, upper_red)
mask = mask0+mask1
imgGoal = cv.bitwise_and(img,img,mask=mask)


# cv.imshow('Obstacle',imgObstacle)
# cv.imshow("Start",imgStart)
# cv.imshow("End",imgGoal)
# cv.waitKey(0)

def getcontours(img):
    imgGray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    imgBlur = cv.GaussianBlur(imgGray,(7,7),1)
    imgCanny = cv.Canny(imgBlur,50,50)
    contours, _ = cv.findContours(imgCanny, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    return contours

def Centre(img):
    contours = getcontours(img)
    print(len(contours))
    for c in contours:
        M = cv.moments(c)
        centre_x = int(M["m10"] / M["m00"])
        centre_y = int(M["m01"] / M["m00"])
    return (centre_x,centre_y)

contours_obstacle = getcontours(imgObstacle)
contours_start = getcontours(imgStart)
contours_goal = getcontours(imgGoal)
start_node = Centre(imgStart)
goal_node = Centre(imgGoal)

rows,columns,ch = img.shape




class RRTStarConnect:
    def __init__(self,start_node,goal_node,img,contours_obstacle,rows,columns):
        self.start_node = start_node
        self.goal_node = goal_node
        # self.start_links = []
        # self.goal_links = []
        self.img = img
        np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)
        self.start = np.array([[(0,0),(0,0),0], [start_node,start_node,0]])
        np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)
        self.goal = np.array([[(0,0),(0,0),0], [goal_node,goal_node,0]])
        np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)
        self.connected = np.array([(0,0),(0,0),(0,0),0,0])
        self.connected_count = 0
        self.rows = rows
        self.cols = columns
        self.radius = 11
        self.delta = 7
        self.contours_obs = contours_obstacle

    def pathplanning(self):
        c=0
        while TRUE:
          cv.imshow("Final",self.img)
          cv.waitKey(1)        
          if(c%2==0):
            flag = 0
            randpt = self.RandomPt()
            x_near,min_dist,xnear_cost = self.NearestNode(randpt,flag)
            x_new = self.Xnew(x_near,randpt,min_dist)
            in_obstacle = self.CollisionDetection(x_new)
            if(in_obstacle==1):
              continue
            near_neighbours = self.NearNeighbours(x_new,flag)
            parentNode,xnew_cost = self.CostEstimate(near_neighbours,x_new,x_near,min_dist,xnear_cost,flag)
            self.linkXnew(x_new,parentNode,flag)
            self.CanConnect(x_new,parentNode,xnew_cost,flag)
            if(self.connected_count >= 1000):
              self.plotPath()
              break
            self.Rewire(near_neighbours,x_new,xnew_cost,flag)
            c=c+1
          else:
            flag = 1
            randpt = self.RandomPt()
            x_near,min_dist,xnear_cost = self.NearestNode(randpt,flag)
            x_new = self.Xnew(x_near,randpt,min_dist)
            in_obstacle = self.CollisionDetection(x_new)
            if(in_obstacle==1):
              continue
            near_neighbours = self.NearNeighbours(x_new,flag)
            parentNode,xnew_cost = self.CostEstimate(near_neighbours,x_new,x_near,min_dist,xnear_cost,flag)
            self.linkXnew(x_new,parentNode,flag)
            self.CanConnect(x_new,parentNode,xnew_cost,flag)
            if(self.connected_count >= 1000):
              self.plotPath()
              break
            self.Rewire(near_neighbours,x_new,xnew_cost,flag)                    
            c=c+1
        cv.destroyAllWindows()

    def RandomPt(self):
        x = random.randint(0,self.cols)
        y = random.randint(0,self.rows)
        coordinate = [x,y]
        return tuple(coordinate)
    
    def NearestNode(self,randPt,flag):
        if(flag==0):
            min_dist = int(math.dist(randPt,self.start_node))
            nearNode = self.start_node
            nearNode_cost = 0
            start_nodes = self.start[1:,:]
            for node,parentNode,cost in start_nodes:
                if(math.dist(randPt,node) < min_dist):
                    min_dist = int(math.dist(randPt,node))
                    nearNode = node
                    nearNode_cost = cost
            return nearNode,min_dist,nearNode_cost
        else:
            min_dist = int(math.dist(randPt,self.goal_node))
            nearNode = self.goal_node
            nearNode_cost = 0
            goal_nodes = self.goal[1:,:]
            for node,parentNode,cost in goal_nodes:
                if(math.dist(randPt,node) < min_dist):
                    min_dist = int(math.dist(randPt,node))
                    nearNode = node
                    nearNode_cost = cost
            return nearNode,min_dist,nearNode_cost           

    def Xnew(self,Xnear,randpt,min_dist):
        x_new = [0,0]
        if(min_dist <= self.delta):
            x_new[0] = int(randpt[0])
            x_new[1] = int(randpt[1])
        else:
            mag = pow(pow(randpt[0]-Xnear[0],2)+pow(randpt[1]-Xnear[1],2),0.5)
            x_new[0] = int((((randpt[0]-Xnear[0])/mag)*self.delta)+Xnear[0])
            x_new[1] = int((((randpt[1]-Xnear[1])/mag)*self.delta)+Xnear[1])

        return tuple(x_new)

    def CollisionDetection(self,x_new):
        in_obstacle = 0
        for contour in self.contours_obs:
            dist = cv.pointPolygonTest(contour,x_new,False)
            if(dist==1):
                in_obstacle = 1
                break
        return in_obstacle

    def CanConnect(self,x_new,parentNode,xnew_cost,flag):
        if(flag==0):
            startTreeNode = parentNode
            goal_nodes = self.goal[1:,:]
            for node,parentNode,cost in goal_nodes:
                if(int(math.dist(x_new,node))<=self.radius):
                    goalTreeNode = node
                    cost_start = xnew_cost
                    cost_goal = cost+int(math.dist(x_new,node))
                    self.connected = np.vstack((self.connected,np.array([x_new,startTreeNode,goalTreeNode,cost_start,cost_goal])))
                    self.connected_count = self.connected_count + 1
                    break                    
        else:
            goalTreeNode = parentNode
            start_nodes = self.start[1:,:]
            for node,parentNode,cost in start_nodes:
                if(int(math.dist(x_new,node))<=self.radius):
                    startTreeNode = node
                    cost_start = cost+int(math.dist(x_new,node))
                    cost_goal = xnew_cost
                    self.connected = np.vstack((self.connected,np.array([x_new,startTreeNode,goalTreeNode,cost_start,cost_goal])))
                    self.connected_count = self.connected_count + 1
                    break              

    def NearNeighbours(self,xnew,flag):
        if(flag==0):    
            neighbours = np.array([[(0,0),0],[(0,0),0]])
            if(math.dist(xnew,self.start_node)<=self.radius):
                np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)
                neighbours = np.vstack((neighbours, np.array([self.start_node,0])))
            start_nodes = self.start[1:,:]
            for node,parentNode,cost in start_nodes:
                if(math.dist(xnew,node) <= self.radius):
                    np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)
                    neighbours = np.vstack((neighbours,np.array([node,cost])))
            return neighbours
        else:
            neighbours = np.array([[(0,0),0],[(0,0),0]])
            if(math.dist(xnew,self.goal_node)<=self.radius):
                np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)
                neighbours = np.vstack((neighbours, np.array([self.goal_node,0])))
            goal_nodes = self.goal[1:,:]
            for node,parentNode,cost in goal_nodes:
                if(math.dist(xnew,node) <= self.radius):
                    np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)
                    neighbours = np.vstack((neighbours,np.array([node,cost])))
            return neighbours
    
    def CostEstimate(self,near_neighbours,x_new,x_near,min_dist,xnear_cost,flag):
        if(flag==0):    
            xnew_cost = xnear_cost + min_dist
            parentNode = x_near
            neighbours = near_neighbours[2:,:]
            for node,cost in neighbours:
                dist = int(math.dist(x_new,node))
                if(cost+dist<xnew_cost):
                    parentNode = node
                    xnew_cost = cost+dist
            self.start = np.vstack((self.start,np.array([x_new,parentNode,xnew_cost])))

            return parentNode,xnew_cost
        else:
            xnew_cost = xnear_cost + min_dist
            parentNode = x_near
            neighbours = near_neighbours[2:,:]
            for node,cost in neighbours:
                dist = int(math.dist(x_new,node))
                if(cost+dist<xnew_cost):
                    parentNode = node
                    xnew_cost = cost+dist
            self.goal = np.vstack((self.goal,np.array([x_new,parentNode,xnew_cost])))

            return parentNode,xnew_cost            

    def linkXnew(self,xnew,parentNode,flag):
        if(flag==0):
            cv.line(self.img,xnew,parentNode,(255,0,0),thickness=1)
        else:
            cv.line(self.img,xnew,parentNode,(0,255,0),thickness=1)

    def Rewire(self,near_neighbours,x_new,xnew_cost,flag):
        if(flag==0):
            neighbours = near_neighbours[2:,:]
            for node,cost in neighbours:
                cost_neighbour = xnew_cost + int(math.dist(x_new,node))
                nodes = self.start[:,0]
                if(cost_neighbour<cost):
                    for i in range(len(nodes)):
                        if (i==0):
                            continue
                        if(nodes[i]==node):
                            b=i
                            break

                    self.start[b,1] = x_new
                    self.linkXnew(node,x_new,flag)
        else:
            neighbours = near_neighbours[2:,:]
            for node,cost in neighbours:
                cost_neighbour = xnew_cost + int(math.dist(x_new,node))
                nodes = self.goal[:,0]
                if(cost_neighbour<cost):
                    for i in range(len(nodes)):
                        if (i==0):
                            continue
                        if(nodes[i]==node):
                            b=i
                            break
                    
                    self.goal[b,1] = x_new
                    self.linkXnew(node,x_new,flag)                

    def plotPath(self):
        minCost = self.connected[1,3] + self.connected[1,4]
        bestNode = self.connected[1,0]
        goalNode = self.connected[1,2]
        startNode = self.connected[1,1]
        connected_nodes= self.connected[1:,:]
        for node,startTreeNode,goalTreeNode,startCost,goalCost in connected_nodes:
            if(startCost+goalCost<minCost):
                minCost = startCost+goalCost
                bestNode = node
                goalNode = goalTreeNode
                startNode = startTreeNode
        cv.line(self.img,bestNode,startNode,(0,0,255),thickness=1)
        cv.line(self.img,bestNode,goalNode,(0,0,255),thickness=1)
        nodes = self.start[:,0]
        while(startNode != self.start_node):
            for i in range(len(nodes)):
                if (i==0):
                    continue                
                if(nodes[i]==startNode):
                    b=i
                    break
            cv.line(self.img,self.start[b,0],self.start[b,1],(0,0,255),thickness=1)
            startNode = self.start[b,1]

        nodes = self.goal[:,0]
        while(goalNode != self.goal_node):
            for i in range(len(nodes)):
                if (i==0):
                    continue                
                if(nodes[i]==goalNode):
                    b=i
                    break
            cv.line(self.img,self.goal[b,0],self.goal[b,1],(0,0,255),thickness=1)
            goalNode = self.goal[b,1]        

        


        
path = RRTStarConnect(start_node,goal_node,img,contours_obstacle,rows,columns)

path.pathplanning()

cv.imshow("Final2",img)
cv.waitKey(0)


