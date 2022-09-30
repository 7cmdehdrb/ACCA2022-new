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
from random import randint
from time import sleep

# msg
from geometry_msgs.msg import *
from nav_msgs.msg import *
from std_msgs.msg import *
from visualization_msgs.msg import *
from erp42_control.msg import *
from path_plan.msg import PathResponse


class Grid(object):
    def __init__(self, x, y, size, count=0):
        self.x = x
        self.y = y
        self.size = size
        self.count = count

    def __add__(self, other):
        return Grid(x=self.x, y=self.y, size=self.size, count=int(self.count + other.count))

    def __str__(self):
        return "Grid(%.2f, %.2f), %d" % (self.x, self.y, self.count)

    def plus(self):
        self.count += 1

    def minus(self):
        self.count += 1


class GridMap(object):
    def __init__(self, xrange=[-100, 100], yrange=[-100, 100], size=0.25, data=None):
        self.xrange = xrange
        self.yrange = yrange
        self.size = size

        if data is None:
            self.map = self.createMap()
        else:
            self.map = data

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

    def createMap(self):
        grid_map = []

        for y in np.arange(self.xrange[0], self.xrange[-1], self.size):
            xs = []
            for x in np.arange(self.xrange[0], self.xrange[-1], self.size):
                grid = Grid(x=x, y=y, size=self.size, count=randint(0, 2))
                xs.append(grid)
            grid_map.append(xs)

        return grid_map


if __name__ == "__main__":
    grid_map = GridMap(xrange=[-2, 2], yrange=[-2, 2],
                       size=1, data=None)
    grid_map2 = GridMap(xrange=[-2, 2], yrange=[-2, 2],
                        size=1, data=None)
