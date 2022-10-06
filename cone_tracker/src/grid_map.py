#!/usr/bin/env python


# Basic
import os
import sys
import rospy
import rospkg
import numpy as np
import math as m
from enum import Enum
from random import randint, random
import tf
from tf.transformations import quaternion_from_euler
from matplotlib import pyplot as plt
from random import randint
from time import sleep

# msg
from geometry_msgs.msg import *
from nav_msgs.msg import *
from std_msgs.msg import *
from visualization_msgs.msg import *
from erp42_control.msg import *
from path_plan.msg import PathResponse


max_cost = 10


class Grid(object):
    def __init__(self, x, y, size, count=0):
        self.x = x
        self.y = y
        self.size = size
        self.count = count

    def __add__(self, other):
        if self.count == max_cost or other.count == max_cost:
            count = max_cost

        else:
            if self.count != 0 and other.count != 0:
                count = int(m.floor(self.count + other.count) / 2)

            elif self.count == 0 and other.count == 0:
                count = 0

            else:
                count = int(self.count if self.count >
                            other.count else other.count)

        return Grid(x=self.x, y=self.y, size=self.size, count=np.clip(count, 0, max_cost))

    def __str__(self):
        return "Grid(%.2f, %.2f), %d" % (self.x, self.y, self.count)

    def add(self, value):
        count = np.clip(self.count + value, 0, max_cost)
        self.count = count

    def set(self, value):
        new_count = np.clip(value, 0, max_cost)
        if self.count < new_count:
            self.count = new_count


class GridMap(object):
    def __init__(self, xrange=[-100, 100], yrange=[-100, 100], size=0.25, obstacles=None, data=None):
        self.xrange = xrange
        self.yrange = yrange
        self.size = size

        if data is not None:
            self.map = data

        else:
            self.map = self.createMap()

            if obstacles is not None:
                self.addObstacles(obstacles)

    def __str__(self):
        text = ""
        for i in range(len(self.map)):
            for j in range(len(self.map[0])):
                text += str(self.map[i][j]) + "\t"
            text += "\n"

        return text

    def __add__(self, other):
        new_map = []

        for i in range(len(self.map)):
            xs = []
            for j in range(len(self.map[0])):
                grid = self.map[i][j] + other.map[i][j]
                xs.append(grid)

            new_map.append(xs)

        return GridMap(xrange=self.xrange, yrange=self.yrange, size=self.size, data=new_map)

    def getGridIdx(self, point=[0., 0.]):
        xs = np.abs(np.arange(self.xrange[0], self.xrange[-1],
                              self.size) - np.array([point[0]]))
        ys = np.abs(np.arange(self.yrange[0], self.yrange[-1],
                              self.size) - np.array([point[1]]))

        return np.argmin(ys), np.argmin(xs)

    def createMap(self):
        grid_map = []

        for y in np.arange(self.yrange[0], self.yrange[-1] + 1, self.size):
            xs = []
            for x in np.arange(self.xrange[0], self.xrange[-1] + 1, self.size):
                grid = Grid(x=x, y=y, size=self.size, count=0)
                xs.append(grid)
            grid_map.append(xs)

        return grid_map

    def addObstacles(self, obstacles):
        for obstacle in obstacles:
            
            i, j = self.getGridIdx(obstacle)

            try:

                for a in range(-10, 11):
                    for b in range(-10, 11):
                        cost = max_cost - int(np.clip(m.sqrt(a ** 2 + b ** 2), 0, 10))
                        if a == 0 and b == 0:
                            cost = max_cost
                        
                        # if a ** 2 + b ** 2 <= 10:
                        self.map[i + a][j + b].set(cost)
                            
                        # self.map[i + a][j + b].add(cost)

            except IndexError:
                pass
            
            except Exception as ex:
                rospy.logfatal(ex)


    def parseOccupiedGrid(self):
        msg = OccupancyGrid()

        xrange = np.arange(self.xrange[0], self.xrange[-1] + 1, self.size)
        yrange = np.arange(self.yrange[0], self.yrange[-1] + 1, self.size)

        data = []

        msg.header = Header(None, rospy.Time.now(), "odom")
        msg.info = MapMetaData(
            rospy.Time.now(), self.size, len(xrange), len(yrange), Pose(
                Point(float(self.xrange[0]), float(self.yrange[0]), 0.0), Quaternion())
        )

        for i in range(len(self.map)):
            for j in range(len(self.map[0])):
                value = self.map[i][j].count
                data.append(int(value * (100 / max_cost)))
                
        for i in range(11):
            data[i] = i * 10

        msg.data = data

        return msg


if __name__ == "__main__":
    xrange = [-100, 100]
    yrange = [-100, 100]

    obstacles = [
        [randint(-80, 80), randint(-80, 80)] for _ in range(100)
    ]

    grid_map = GridMap(xrange, yrange, size=1, obstacles=obstacles)

    grid_map.plot()
    plt.show()
