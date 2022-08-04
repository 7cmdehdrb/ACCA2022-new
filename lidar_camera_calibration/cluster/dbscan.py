#!/usr/bin/env python2.7


import rospy
import math
import numpy as np
import tf2_ros
import enum
from geometry_msgs.msg import PointStamped, PoseArray, Pose, PoseStamped
from nav_msgs.msg import Path
from operator import pos
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

class DBSCAN(object):
    def __init__(self, epsilon, minpts):
        self.epsilon = epsilon
        self.minpts = minpts
        self.C = 0
        self.point_data = []
        self.x = []
        self.data = []
        self.centerpts = []
        self.degree = 0

    def pointcloudCallback(self, msg):
        self.point_data = []
        for mark in msg.markers:
            for j in range(len(mark.points)):
                temp = [mark.points[j].x,mark.points[j].y,mark.points[j].z]
                self.point_data.append(temp)


    def cvtRange(self):
        if len(self.point_data) == 0:
            pass

        else:
            self.x = []

            self.x = self.point_data
            self.data = np.array(self.x)
            print(self.data)


    def run(self):
        if not len(self.data) == 0:
            self.n = len(self.data)
            # Euclidean distance
            p, q = np.meshgrid(np.arange(self.n), np.arange(self.n))
            self.dist = np.sqrt(np.sum(((self.data[p] - self.data[q])**2), 2))
            # label as visited points and noise
            self.visited = np.full((self.n), False)
            self.noise = np.full((self.n), False)
            self.idx = np.full((self.n), 0)
            self.input = self.data
            # Clustering
            for i, vector in enumerate(self.data):
                if self.visited[i] == False:
                    self.visited[i] = True
                    self.neighbors = self.regionQuery(i)
                    if len(self.neighbors) > self.minpts:
                        self.C += 1
                        self.expandCluster(i)
                    else:
                        self.noise[i] = True

            return self.idx, self.noise

    def regionQuery(self, i):
        g = self.dist[i, :] < self.epsilon
        Neighbors = np.where(g == True)[0].tolist()

        return Neighbors

    def expandCluster(self, i):
        self.idx[i] = self.C
        k = 0

        while True:
            try:
                j = self.neighbors[k]
            except:
                pass
            if self.visited[j] != True:
                self.visited[j] = True

                self.neighbors2 = self.regionQuery(j)

                if len(self.neighbors2) > self.minpts:
                    self.neighbors = self.neighbors+self.neighbors2

            if self.idx[j] == 0:
                self.idx[j] = self.C

            k += 1
            if len(self.neighbors) < k:
                return

    def sort(self):
        self.cluster = []
        self.noise = []
        if not len(self.data) == 0:
            cnum = np.max(self.idx)

            for i in range(cnum):

                k = np.where(self.idx == (i+1))[0].tolist()
                
                if not len(k) == 0:
                    self.cluster.append([self.input[k, :]])

            self.noise = self.input[np.where(self.idx == 0)[0].tolist(), :]

        return self.cluster, self.noise

    def find_far_pt(self, cluster):
        self.centerpts = []
        for idx, group in enumerate(cluster):
            pt = np.mean(cluster[idx][0], axis=0).tolist()
            if math.sqrt(math.pow(pt[1],2) + math.pow(pt[0],2)+math.pow(pt[2],2)) < 10.0: 

            # print((np.sqrt(pow((group[0][:,0]-(np.mean(cluster[idx][0],axis=0)[0])),2)+pow((group[0][:,1]-(np.mean(cluster[idx][0],axis=0)[1])),2))))
                self.centerpts.append(np.mean(cluster[idx][0], axis=0).tolist())
        
        
        
        return self.centerpts


if __name__ == "__main__":
    rospy.init_node("dbscan")

    dbscan = DBSCAN(0.5, 2)

    rospy.Subscriber("/before_adaptive", MarkerArray, dbscan.pointcloudCallback)
    pub = rospy.Publisher("/cluster_complete", MarkerArray, queue_size=1)

    position = []

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():

        dbscan.cvtRange()

        dbscan.run()
        cluster, _ = dbscan.sort()

        obstaclearray = MarkerArray()

        if not len(cluster) == 0:
        
    
            position = dbscan.find_far_pt(cluster)
            obstacle = Marker()
            obstacle.header.frame_id = "velodyne"
            obstacle.ns = "points"
            obstacle.id = 1
            obstacle.type = 8
            obstacle.action = 0
            obstacle.color = ColorRGBA(0,0,1,1)
            obstacle.scale.x = 0.01
            obstacle.scale.y = 0.01
            obstacle.scale.z = 0

            for i in range(len(position)): 
                obstacle.points.append(Point(position[i][0],position[i][1],position[i][2]))

            obstaclearray.markers.append(obstacle)

        pub.publish(obstaclearray)

        r.sleep()