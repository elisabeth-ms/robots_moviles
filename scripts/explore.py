#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import numpy as np
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import random
from scipy.cluster.vq import vq, kmeans,whiten
import tf
class Explore:
    def __init__(self):
        rospy.Subscriber("map", OccupancyGrid, self.callback_map)
        rospy.Subscriber("move_base/result", MoveBaseActionResult, self.callback_result)
        self.pub_vel = rospy.Publisher("/mobile_base/commands/velocity",Twist)
        self.pub_goal = rospy.Publisher("move_base_simple/goal",PoseStamped)

        self.map =[]
        self.res = 1
        self.fronteirs=[]
        self.origin_x = 0
        self.origin_y = 0
        self.goal_reached = True
        self.init = True
        self.rate = rospy.Rate(1.0)  # 10hz
        self.goal = PoseStamped()
        self.end = False
        self.use_random_cell = False
        self.current_x = 0
        self.current_y = 0
        t = Twist()
        t.angular.z = 0.7
        self.listener = tf.TransformListener()

        while not rospy.is_shutdown():
            if self.init:
                for i in range(16):
                    self.pub_vel.publish(t)
                    self.rate.sleep()
                    if i == 15:
                        self.init = False

            print "Current:",self.current_x,self.current_y
            rospy.spin()


    def readMap(self, map):
        for row in map:
            print row
    def callback_result(self,data):
        if data.status.status==3:
            #print data.status.status
            self.goal_reached = True
    def callback_map(self,data):
        self.res = data.info.resolution
        #print self.res
        self.map = np.zeros((data.info.width, data.info.height), dtype=int)
        self.origin_x = data.info.origin.position.x
        self.origin_y = data.info.origin.position.y
        #print self.origin_x, self.origin_y
        for i in range(0, data.info.width):
            for j in range(0, data.info.height):
                # print data.data[j * data.info.width + i]
                    self.map[i][j] = data.data[j * data.info.width + i]
        #self.readMap(self.map)
        #Busco que celdas del mapa estan libres y cerca de celdas que no se sabe su estado
        if not self.end:
            if not self.init and not self.goal_reached:
                if self.check_closer_obstacles(int((self.goal.pose.position.x-self.origin_x)/self.res),int((self.goal.pose.position.y-self.origin_y)/self.res)):
                    self.goal_reached=True
                    print "Impossible to reach this goal"
                if self.map[int((self.goal.pose.position.x-self.origin_x)/self.res)][int((self.goal.pose.position.y-self.origin_y)/self.res)] == 0:
                    self.goal_reached = True
                    print "Goal is free"
            if not self.init and self.goal_reached:
                if self.use_random_cell:
                    self.fronteirs = []
                    for i in range(0, data.info.width):
                        for j in range(0, data.info.height):
                            if self.map[i][j] == -1 and self.check_neighbors(i, j) and not self.check_closer_obstacles(i, j):
                                self.fronteirs.append([i, j])
                else:
                     (trans, rot) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                     self.current_x = trans[0]
                     self.current_y = trans[1]
                     current_cell_x = int((self.current_x-self.origin_x)/self.res)
                     current_cell_y = int((self.current_y- self.origin_y)/ self.res)
                     print "Current cell:",current_cell_x,current_cell_y
                     self.fronteirs=[]
                     dmin = 100000000
                     for i in range(0, data.info.width):
                         for j in range(0, data.info.height):
                             if self.map[i][j] == -1 and self.check_neighbors(i, j) and not self.check_closer_obstacles(
                                 i, j):
                                 d = self.distance(current_cell_x,current_cell_y,i,j)
                                 if d <=dmin:
                                    if self.fronteirs:
                                        self.fronteirs.pop(0)
                                    self.fronteirs.append([i,j])
                                    dmin = d

                if not self.fronteirs:
                    self.end = True
                    self.goal_reached = True
                    print "Complete Area scanned"
                else:
                    print self.fronteirs
                    if self.use_random_cell:
                        index = random.randint(0, len(self.fronteirs) - 1)
                    else:
                        index = 0
                    print "Assigning new goal:"
                    print self.fronteirs[index]
                    self.goal.header.frame_id = "map"
                    self.goal.pose.position.x = self.fronteirs[index][0] * self.res + self.origin_x
                    self.goal.pose.position.y = self.fronteirs[index][1] * self.res + self.origin_y
                    print self.goal.pose.position.x, self.goal.pose.position.y
                    self.goal.pose.orientation.w = 1
                    self.pub_goal.publish(self.goal)
                    self.goal_reached = False


    def distance(self, current_cell_x, current_cell_y,fronteir_cell_x,fronteir_cell_y):
        return (current_cell_x - fronteir_cell_x) * (current_cell_x - fronteir_cell_x) + (current_cell_y - fronteir_cell_y) * (current_cell_y - fronteir_cell_y)
    def check_neighbors(self,x,y):
        #Check if a neighbor is free
        if x+1 >= len(self.map[0]) or y+1>= len(self.map) or x-1<=0 or y-1<=0:
            return False
        if self.map[x+1][y]==0:
            return True
        if self.map[x-1][y]==0:
            return True
        if self.map[x][y+1]==0:
            return True
        if self.map[x][y-1]==0:
            return True
        if self.map[x][y]==0:
            return True
        return False

    def check_closer_obstacles(self,x,y):
        if x>=len(self.map[0]) or x==0 or y>=len(self.map) or y==0:
            return True
        if x+1 > len(self.map[0]) or y+1 > len(self.map) or x-1<=0 or y-1<=0:
            return True
        if self.map[x + 1][y] > 0:
            return True
        if self.map[x + 1][y+1] > 0:
            return True
        if self.map[x + 1][y-1] > 0:
            return True
        if self.map[x][y] > 0:
            return True
        if self.map[x][y+1] > 0:
            return True
        if self.map[x][y-1] > 0:
            return True
        if self.map[x-1][y] > 0:
            return True
        if self.map[x-1][y+1] > 0:
            return True
        if self.map[x-1][y-1] > 0:
            return True
        return False
if __name__ == '__main__':
    rospy.init_node('explore_node', anonymous=True)
    try:
        explore = Explore()
    except rospy.ROSInterruptException:
        pass
