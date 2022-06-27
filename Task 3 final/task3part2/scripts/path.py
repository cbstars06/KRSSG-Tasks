#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty
import math
import time
from pickle import FALSE, TRUE
import random
import cv2 as cv
import numpy as np

class Image:
    def __init__(self,img):
        self.img = img
        self.imgObstacle = None
        self.imgStart = None
        self.imgGoal = None
        self.contours_obstacle = None
        self.contours_start = None
        self.contours_goal = None
        self.start_node = None
        self.goal_node = None
        self.rows = img.shape[0]
        self.columns = img.shape[1]

    def imageHandling(self):
        imgHSV = cv.cvtColor(self.img,cv.COLOR_BGR2HSV)

        #Goal Image
        lower_green = np.array([45,100,20])
        upper_green = np.array([75,255,255])
        mask_start = cv.inRange(imgHSV,lower_green,upper_green)
        self.imgGoal = cv.bitwise_and(self.img,self.img,mask=mask_start)

        #Start Image
        lower_red = np.array([0,50,50])
        upper_red = np.array([10,255,255])
        mask0 = cv.inRange(imgHSV, lower_red, upper_red)
        # upper mask (170-180)
        lower_red = np.array([170,50,50])
        upper_red = np.array([180,255,255])
        mask1 = cv.inRange(imgHSV, lower_red, upper_red)
        mask = mask0+mask1
        self.imgStart = cv.bitwise_and(self.img,self.img,mask=mask)

        #Black Border
        border = np.zeros((self.rows,self.columns), dtype=np.uint8)
        cv.rectangle(border,(7,7),(self.columns-7,self.rows-7), (255,255,255) ,thickness = -1)

        #Obstacle Image
        mask_obs = cv.bitwise_not(mask_start+mask)
        imgObstacle = cv.bitwise_and(self.img,self.img,mask=mask_obs)
        self.imgObstacle = cv.bitwise_and(imgObstacle,imgObstacle,mask=border)

    def getcontours(self,img):
        imgGray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
        imgBlur = cv.GaussianBlur(imgGray,(7,7),1)
        imgCanny = cv.Canny(imgBlur,50,50)
        imgDialated = cv.dilate(imgCanny,(7,7), iterations=3)
        contours, _ = cv.findContours(imgDialated, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        return contours

    def Centre(self,contour):
        c = contour
        M = cv.moments(c)
        centre_x = int(M["m10"] / M["m00"])
        centre_y = int(M["m01"] / M["m00"])
        return (centre_x,centre_y)
    
    def update(self):
        self.imageHandling()
        self.contours_obstacle = self.getcontours(self.imgObstacle)
        self.contours_start = self.getcontours(self.imgStart)
        self.contours_goal = self.getcontours(self.imgGoal)
        start_num = int(input("Enter start contour no.: "))
        goal_num = int(input("Enter goal contour no.: "))
        self.start_node = self.Centre(self.contours_start[start_num-1])
        self.goal_node = self.Centre(self.contours_goal[goal_num-1])

class Node:
    def __init__(self,coordinates,cost=None):
        self.coordinate = coordinates
        self.cost = cost
        self.childs = np.array([])
        self.parent = None

class connectedNode:
    def __init__(self,coordinates,parent_start,parent_goal,start_cost,goal_cost):
        self.coordinate = coordinates
        self.parentStart = parent_start
        self.parentGoal = parent_goal
        self.start_cost = start_cost
        self.goal_cost = goal_cost

class pathCoordinate:
    def __init__(self,coordinates):
        self.coordinate = coordinates
        self.parent = None

class RRTStarConnect:
    def __init__(self,start_node,goal_node,img,contours_obstacle,rows,columns):
        self.start_node = Node(start_node,0)
        self.start_node.parent = self.start_node
        self.goal_node = Node(goal_node,0)
        self.goal_node = self.goal_node
        self.img = img
        np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)
        self.start = np.array([self.start_node])
        np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)
        self.goal = np.array([self.goal_node])
        np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)
        self.connected = np.array([])
        self.connected_count = 0
        self.rows = rows
        self.cols = columns
        self.radius = 40
        self.delta = 30
        self.check_radius = 4
        self.contours_obs = contours_obstacle
        self.start_path = np.array([])
        self.goal_path = np.array([])
        self.connection = None


    def pathplanning(self):
        c=0
        while TRUE:
            cv.imshow("Final",self.img)
            cv.waitKey(1)        
            if(c%2==0):
              flag = 0
            else:
              flag = 1
            randpt = self.RandomPt()
            nearNode,min_dist = self.NearestNode(randpt,flag)
            x_new = self.Xnew(nearNode,randpt,self.delta)
            in_obstacle = self.CollisionDetection(x_new)
            if(in_obstacle==1):
              continue
            near_neighbours = self.NearNeighbours(x_new,flag)
            neighbour_success = self.CostEstimate(near_neighbours,x_new,nearNode,min_dist,flag)
            if(neighbour_success == -1):
                continue
            self.linkXnew(x_new,flag)
            self.CanConnect(x_new,flag)
            if(self.connected_count >= 100):
              self.plotPath()
              cv.destroyAllWindows()
              break
            self.Rewire(near_neighbours,x_new,flag)
            c=c+1

        return self.connection,self.start_path,self.goal_path
        
    
    def RandomPt(self):
        x = random.randint(0,self.cols)
        y = random.randint(0,self.rows)
        coordinate = [x,y]
        return tuple(coordinate)
    
    def NearestNode(self,randPt,flag):
        min_dist = int(math.dist(randPt,self.start_node.coordinate))
        if(flag==0):
            nearNode = self.start_node
            arr = self.start
        else:
            nearNode = self.goal_node
            arr = self.goal
        for node in arr:
            if(math.dist(randPt,node.coordinate) < min_dist):
                min_dist = int(math.dist(randPt,node.coordinate))
                nearNode = node
        return nearNode,min_dist         

    def Xnew(self,nearNode,randpt,fix_dist):
        min_dist = int(math.dist(nearNode.coordinate,randpt))
        x_new = Node((0,0))
        if(min_dist <= fix_dist):
            x_new.coordinate = (int(randpt[0]),int(randpt[1]))
        else:
            mag = pow(pow(randpt[0]-nearNode.coordinate[0],2)+pow(randpt[1]-nearNode.coordinate[1],2),0.5)
            x = int((((randpt[0]-nearNode.coordinate[0])/mag)*fix_dist)+nearNode.coordinate[0])
            y = int((((randpt[1]-nearNode.coordinate[1])/mag)*fix_dist)+nearNode.coordinate[1])
            x_new.coordinate = (x,y)
        return x_new

    def CollisionDetection(self,x_new):
        in_obstacle = 0
        for contour in self.contours_obs:
            dist = cv.pointPolygonTest(contour,x_new.coordinate,False)
            if(dist==1 or dist==0):
                in_obstacle = 1
                break
        return in_obstacle

    def EdgeCollisionCheck(self,node,purpose,parent = None):
        if(purpose == 0):
            in_obstacle = 0
            c = self.Xnew(parent,node.coordinate,self.check_radius)
            while(math.dist(parent.coordinate,c.coordinate)<math.dist(node.coordinate,parent.coordinate)):
                in_obstacle = self.CollisionDetection(c)
                if(in_obstacle==1):
                    return in_obstacle
                c = self.Xnew(c,node.coordinate,self.check_radius)

            return in_obstacle

        elif(purpose == 1):
            in_obstacle = 0
            c = self.Xnew(node.parentGoal,node.coordinate,self.check_radius)
            while(math.dist(c.coordinate,node.parentGoal.coordinate)<math.dist(node.coordinate,node.parentGoal.coordinate)):
                in_obstacle = self.CollisionDetection(c)
                if(in_obstacle==1):
                    return in_obstacle
                c = self.Xnew(c,node.coordinate,self.check_radius)

            return in_obstacle

        elif(purpose == 2):
            in_obstacle = 0
            c = self.Xnew(node.parentStart,node.coordinate,self.check_radius)
            while(math.dist(c.coordinate,node.parentStart.coordinate)<math.dist(node.coordinate,node.parentStart.coordinate)):
                in_obstacle = self.CollisionDetection(c)
                if(in_obstacle==1):
                    return in_obstacle
                c = self.Xnew(c,node.coordinate,self.check_radius)

            return in_obstacle             

        elif(purpose == 3):
            in_obstacle = 0
            c = self.Xnew(parent,node.coordinate,self.check_radius)
            while(math.dist(parent.coordinate,c.coordinate)<math.dist(parent.coordinate,node.coordinate)):
                in_obstacle = self.CollisionDetection(c)
                if(in_obstacle==1):
                    return in_obstacle
                c = self.Xnew(c,node.coordinate,self.check_radius)               

            return in_obstacle     
        
    def CanConnect(self,x_new,flag):
        if(flag==0):
            for node in self.goal:
                if(int(math.dist(x_new.coordinate,node.coordinate))<=self.radius):
                    cost_goal = node.cost + int(math.dist(x_new.coordinate,node.coordinate))
                    connected_node = connectedNode(x_new.coordinate,x_new.parent,node,x_new.cost,cost_goal)
                    if(self.EdgeCollisionCheck(connected_node,1)==1):
                        continue
                    self.connected = np.append(self.connected,connected_node)
                    self.connected_count = self.connected_count + 1
                    break                    
        else:
            for node in self.start:
                if(int(math.dist(x_new.coordinate,node.coordinate))<=self.radius):
                    cost_start = node.cost + int(math.dist(x_new.coordinate,node.coordinate))
                    connected_node = connectedNode(x_new.coordinate,node,x_new.parent,cost_start,x_new.cost)
                    if(self.EdgeCollisionCheck(connected_node,2)==1):
                        continue                    
                    self.connected = np.append(self.connected,connected_node)
                    self.connected_count = self.connected_count + 1
                    break                  

    def NearNeighbours(self,xnew,flag):
        if(flag==0):
            arr = self.start
        else:
            arr = self .goal    
        neighbours = np.array([])
        for node in arr:
            if(math.dist(xnew.coordinate,node.coordinate) <= self.radius):
                np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)
                neighbours = np.append(neighbours,node)
        return neighbours
    
    def CostEstimate(self,near_neighbours,x_new,x_near,min_dist,flag):
        l = 0   
        x_new.cost = x_near.cost + min_dist
        x_new.parent = x_near
        for node in near_neighbours:
            dist = int(math.dist(x_new.coordinate,node.coordinate))
            if(node.cost+dist<=x_new.cost):
                edge_collision_check = self.EdgeCollisionCheck(x_new,0,node)
                if(edge_collision_check == 1):
                    continue
                x_new.parent = node
                x_new.cost = node.cost + dist
                l=1
        
        if(l==0):
            return -1

        x_new.parent.childs = np.append(x_new.parent.childs,x_new)
        if(flag==0):
            self.start = np.append(self.start,x_new)
        else:
            self.goal = np.append(self.goal,x_new)          

        return 1

    def linkXnew(self,x_new,flag):
        if(flag==0):
            cv.line(self.img,x_new.coordinate,x_new.parent.coordinate,(255,0,0),thickness=1)
        else:
            cv.line(self.img,x_new.coordinate,x_new.parent.coordinate,(0,255,0),thickness=1)

    def Rewire(self,near_neighbours,x_new,flag):
        for node in near_neighbours:
            cost_neighbour = x_new.cost + int(math.dist(x_new.coordinate,node.coordinate))
            if(cost_neighbour<node.cost):
                if (self.EdgeCollisionCheck(node,3,x_new)==1):
                    continue
                childs = node.parent.childs
                for i in range(len(childs)):
                    if(childs[i]==node):
                        np.delete(childs, i)
                        break
                node.parent = x_new
                node.cost = cost_neighbour
                x_new.childs = np.append(x_new.childs,node)
                self.newCost(node)
                self.linkXnew(node,flag)
             
    def newCost(self,node):
        childs = node.childs
        for child_node in childs:
            child_node.cost = node.cost + int(math.dist(child_node.coordinate,node.coordinate))

    def plotPath(self):
        minCost = self.connected[0].start_cost + self.connected[0].goal_cost
        bestNode = self.connected[0]
        for node in self.connected:
            if(node.start_cost+ node.goal_cost<minCost):
                minCost = node.start_cost+node.goal_cost
                bestNode = node

        cv.line(self.img,bestNode.coordinate,bestNode.parentStart.coordinate,(0,0,255),thickness=2)
        cv.line(self.img,bestNode.coordinate,bestNode.parentGoal.coordinate,(0,0,255),thickness=2)
        self.connection = bestNode
        node = bestNode.parentStart
        while(node != self.start_node):
            cv.line(self.img,node.coordinate,node.parent.coordinate,(0,0,255),thickness=2)
            self.start_path = np.append(self.start_path,node)
            node = node.parent
        self.start_path = np.append(self.start_path,self.start_node)
        
        node = bestNode.parentGoal
        while(node != self.goal_node):
            cv.line(self.img,node.coordinate,node.parent.coordinate,(0,0,255),thickness=2)
            self.goal_path = np.append(self.goal_path,node)
            node = node.parent 
        self.goal_path = np.append(self.goal_path,self.goal_node)

class TurtleMove:
    def __init__(self,turtlepath):
        self.turtle_position = None
        self.current_theta = None
        self.turtleThetaNew = 0
        self.turtlepath = turtlepath
        self.current_coordinate = turtlepath[0].coordinate

    def calcTheta(self,node1,node2):
        if(node2[0] > node1[0]):
            theta = math.atan((node2[1]-node1[1])/(node2[0]-node1[0]))
        elif(node2[0] < node1[0] and node2[1]<node1[1]):
            theta = -(22/7) + math.atan((node2[1]-node1[1])/(node2[0]-node1[0]))
        elif(node2[0] < node1[0] and node2[1]>node1[1]):
            theta = (22/7) + math.atan((node2[1]-node1[1])/(node2[0]-node1[0]))
        elif(node2[1]>node1[1] and node2[0] == node1[0]):
            theta = 22/14
        else:
            theta = -22/14
        return theta

    def PIDOrientation(self,velocity_publisher):
        velocity_message = Twist()
        integral_term = 0
        differential_term = 0
        angular_speed = 0
        itr = 0 
        rate = rospy.Rate(8)
        while TRUE:
            while self.current_theta == None:
                rate.sleep()
            t1 = time.time()
            angle_diff1 = self.turtleThetaNew-self.current_theta
            if (itr==0):
                pass
            else:
                differential_term = (angle_diff1-angle_diff2)/(t1-t2)
                integral_term += angle_diff1*(t1-t2)
                if(abs(angle_diff1)<0.03 and abs(angular_speed)<0.1):
                    break
            print(angle_diff1)

            t2 = time.time()
            angle_diff2 = angle_diff1
            Kp = 3
            Ki = 4
            Kd = 0.0825

            angular_speed = Kp*angle_diff1 + Ki*integral_term + Kd*differential_term
            print(f'Prop - {Kp*angle_diff1}')
            print(f'Int - {Ki*integral_term}')
            print(f'Diff - {Kd*differential_term}')
            velocity_message.angular.z = angular_speed
            print(f'Angular speed: {angular_speed}')
            velocity_publisher.publish(velocity_message)
            rate.sleep()
            itr += 1

    def poseDealing(self,pose_message):
        x = pose_message.x
        y = pose_message.y
        self.turtle_position = (x,y)
        self.current_theta = pose_message.theta

    def PIDVelocity(self,velocity_pub,start,goal):
        velocity_message = Twist()
        integral_term = 0
        differential_term = 0
        linear_speed = 0
        itr = 0
        rate = rospy.Rate(8) 
        while TRUE:
            while self.turtle_position == None:
                rate.sleep()
            t1 = time.time()
            if(math.dist(start.coordinate,self.turtle_position)<math.dist(goal.coordinate,start.coordinate)):
                distance1 = math.dist(goal.coordinate,self.turtle_position)
            else:
                distance1 = -math.dist(goal.coordinate,self.turtle_position)
            
            if (itr==0):
                pass
            else:
                differential_term = (distance1-distance2)/(t1-t2)
                integral_term += distance1*(t1-t2)
                if(abs(distance1)<0.03 and abs(linear_speed)<0.1):
                    self.current_coordinate = self.turtle_position
                    break

            print(f"Distance = {distance1}")
            t2 = time.time()
            distance2 = distance1
            Kp = 4.2
            Ki = 5.633
            Kd = 0.0468

            linear_speed = Kp*distance1 + Ki*integral_term + Kd*differential_term
            velocity_message.linear.x = linear_speed
            velocity_pub.publish(velocity_message)
            print(f'Linear speed: {linear_speed}')
            rate.sleep()
            itr +=1

    def run(self,velocity_publisher):
        for i in range(len(self.turtlepath)-1):
            self.turtleThetaNew = self.calcTheta(self.current_coordinate,self.turtlepath[i+1].coordinate)
            self.PIDOrientation(velocity_publisher)
            self.PIDVelocity(velocity_publisher,self.turtlepath[i],self.turtlepath[i+1])


def TurtlePath(start_path,goal_path,connected_node,rows,columns):
        pathNew = np.array([])
        arr = start_path
        k = len(arr)
        for i in range(len(arr)):
            x1 = arr[k-1-i].coordinate[0]*(11.08889/columns)
            y1 = 11.08889 - arr[k-1-i].coordinate[1]*(11.08889/rows)
            node = pathCoordinate((x1,y1))
            pathNew = np.append(pathNew,node)

        x1 = connected_node.coordinate[0]*(11.08889/columns)
        y1 = 11.08889 - connected_node.coordinate[1]*(11.08889/rows)
        node = pathCoordinate((x1,y1))
        pathNew = np.append(pathNew,node)
        arr = goal_path
        k = len(arr)
        for i in range(len(arr)):
            x1 = arr[i].coordinate[0]*(11.08889/columns)
            y1 = 11.08889 - arr[i].coordinate[1]*(11.08889/rows)
            node = pathCoordinate((x1,y1))
            pathNew = np.append(pathNew,node)

        return pathNew

def handleTeleport(x,y,theta):
    rospy.wait_for_service('/turtle1/teleport_absolute')
    try:
        teleport_turtle = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        server_response = teleport_turtle(x, y, theta)
        return
    except rospy.ServiceException as e:
        print("Service call failes: %s"%e)

def clear_bg():
    rospy.wait_for_service('/clear')
    try:
        clear_bg = rospy.ServiceProxy('/clear',Empty)
        server_response = clear_bg()
        return
    except rospy.ServiceException as e:
        print("Service call failes: %s"%e)

if __name__ == '__main__':
    try:
        rospy.init_node('path_planner', anonymous=True)
               
        img =cv.imread('catkin_ws/src/task3part2/scripts/task3.2.png')
        myImage = Image(img)
        myImage.update()

        path = RRTStarConnect(myImage.start_node,myImage.goal_node,img,myImage.contours_obstacle,myImage.rows,myImage.columns)
        connected_node,start_path,goal_path = path.pathplanning()
        turtlepath = TurtlePath(start_path,goal_path,connected_node,myImage.rows,myImage.columns)

        myturtle = TurtleMove(turtlepath)

        handleTeleport(turtlepath[0].coordinate[0],turtlepath[0].coordinate[1],0)
        clear_bg()

        position_topic ='/turtle1/pose'
        position_subscriber = rospy.Subscriber(position_topic,Pose,myturtle.poseDealing)
        
        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)


        myturtle.run(velocity_publisher)

        cv.imshow("Final",img)
        cv.waitKey(0)


    except rospy.ROSInterruptException:
        rospy.loginfo("Reached Goal")